/*
 * SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Jon Schulman
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "bridge.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ble_kiss.h"
#include "cat.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "nvs.h"
#include "radio_usb.h"
#include "sdkconfig.h"

static const char *TAG = "bridge";

#define CAT_TIMEOUT_MS     500
#define CAT_ATTEMPTS       3
#define WAKE_ATTEMPTS      20   /* DTR pulses before giving up */
#define SETTLE_MS          500  /* after USB open, before the first DTR pulse */
#define DTR_LOW_MS         250
#define DTR_HIGH_MS        250
#define KISS_EXIT_MS       300  /* after C0 FF C0, before talking CAT again */
#define RETRY_PERIOD_MS    5000 /* re-try a radio that did not answer */
#define APP_TX_BUF_SIZE    4096 /* app -> radio; several queued frames */
#define APP_TX_WAIT_MS     5000 /* how long app data may wait for KISS to come up */

#define NVS_NAMESPACE      "kklink"
#define NVS_KEY_ORIG_TN    "orig_tn"

#define BIT_KISS_ACTIVE    BIT0
#define BIT_QSY_DONE       BIT1

#define KISS_FEND          0xC0
#define KISS_FESC          0xDB
#define KISS_TFEND         0xDC
#define KISS_TFESC         0xDD
#define KISS_CMD_SETHW     0x06
#define KISS_FRAME_MAX     1100 /* > 2 * 329 + 2 escaped worst case */
#define QSY_WAIT_MS        15000

/*
 * EXPERIMENT (CONFIG_KK_LINK_QSY_EXPERIMENT): frequency control as observed
 * from RadioMail talking to an adapter named "B.B. Link". It sends KISS
 * SetHardware frames:
 *   C0 06 EA f3 f2 f1 f0 C0   QSY: set the TNC band to frequency f (Hz, big-endian)
 *   C0 06 EB C0               sent before disconnecting; taken as "restore frequency"
 *   C0 06 F4 00 C0            meaning unknown; logged and ignored
 * The D750's own TNC ignores these, so the bridge translates them to CAT.
 */
#define SETHW_QSY          0xEA
#define SETHW_QSY_RESTORE  0xEB

static const uint8_t KISS_EXIT[] = { 0xC0, 0xFF, 0xC0 };

typedef enum {
    EV_USB_UP,
    EV_USB_DOWN,
    EV_SESSION_OPEN,
    EV_SESSION_CLOSE,
    EV_QSY,         /* arg: frequency in Hz */
    EV_QSY_RESTORE,
} bridge_event_t;

typedef struct {
    bridge_event_t type;
    uint32_t arg;
} bridge_msg_t;

static QueueHandle_t s_events;
static EventGroupHandle_t s_flags;
static StreamBufferHandle_t s_app_tx;

/* Routing flag read by the USB RX callback: true = radio bytes go to BLE. */
static volatile bool s_kiss_active;

/* Control task state. */
static bool s_usb_up;
static bool s_radio_ok;  /* answered CAT since the last USB_UP */
static bool s_want_kiss; /* an app is subscribed */
static bool s_in_kiss;   /* we put the radio's TNC into KISS */
static char s_orig_tn[16];
static uint32_t s_dropped_to_app;
static bool s_qsy_saved;     /* s_orig_vm / s_orig_fq hold the pre-QSY state */
static char s_orig_vm[16];   /* e.g. "VM 1,1" */
static char s_orig_fq[24];   /* e.g. "FQ 1,0144390000" (VFO frequency) */

/* ---------- callbacks (other tasks' context) ---------- */

static bool post_arg(bridge_event_t type, uint32_t arg)
{
    const bridge_msg_t msg = { .type = type, .arg = arg };
    if (xQueueSend(s_events, &msg, 0) != pdTRUE) {
        ESP_LOGE(TAG, "event queue full, lost event %d", type);
        return false;
    }
    return true;
}

static void post(bridge_event_t type)
{
    post_arg(type, 0);
}

static void on_radio_rx(const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (s_kiss_active) {
        size_t sent = ble_kiss_send(data, len);
        if (sent < len) {
            s_dropped_to_app += len - sent;
        }
    } else {
        cat_feed(data, len);
    }
}

static void on_radio_state(bool connected, void *ctx)
{
    (void)ctx;
    post(connected ? EV_USB_UP : EV_USB_DOWN);
}

static void on_app_rx(const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
#if CONFIG_KK_LINK_LOG_APP_WRITES
    ESP_LOGI(TAG, "app wrote %u bytes%s", (unsigned)len, s_kiss_active ? "" : " (KISS not active)");
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, len < 64 ? len : 64, ESP_LOG_INFO);
#endif
    size_t n = xStreamBufferSend(s_app_tx, data, len, 0);
    if (n < len) {
        ESP_LOGW(TAG, "app TX buffer full, dropped %u bytes", (unsigned)(len - n));
    }
}

static void on_app_session(bool active, void *ctx)
{
    (void)ctx;
    post(active ? EV_SESSION_OPEN : EV_SESSION_CLOSE);
}

/* ---------- NVS: original TNC mode ---------- */

static bool nvs_load_orig(char *out, size_t size)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = size;
    esp_err_t err = nvs_get_str(h, NVS_KEY_ORIG_TN, out, &len);
    nvs_close(h);
    return err == ESP_OK;
}

static void nvs_store_orig(const char *value)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (value) {
        nvs_set_str(h, NVS_KEY_ORIG_TN, value);
    } else {
        nvs_erase_key(h, NVS_KEY_ORIG_TN);
    }
    nvs_commit(h);
    nvs_close(h);
}

/* ---------- CAT helpers ---------- */

/* A reply belongs to `cmd` if it echoes the 2-letter command or is "?". */
static bool reply_matches(const char *cmd, const char *reply)
{
    return strcmp(reply, "?") == 0 || strncmp(reply, cmd, 2) == 0;
}

static esp_err_t cat_query(const char *cmd, char *reply, size_t reply_size)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= CAT_ATTEMPTS; attempt++) {
        err = cat_command(cmd, reply, reply_size, CAT_TIMEOUT_MS);
        if (err == ESP_OK && reply_matches(cmd, reply)) {
            return ESP_OK;
        }
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "%s: ignoring stray reply '%s'", cmd, reply);
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    return err;
}

/* "TN p1,p2" -> true if well formed. */
static bool parse_tn(const char *reply, char *mode, char *band)
{
    if (strlen(reply) != 6 || strncmp(reply, "TN ", 3) != 0 || reply[4] != ',') {
        return false;
    }
    *mode = reply[3];
    *band = reply[5];
    return true;
}

static void set_kiss_active(bool active)
{
    s_kiss_active = active;
    if (active) {
        xEventGroupSetBits(s_flags, BIT_KISS_ACTIVE);
    } else {
        xEventGroupClearBits(s_flags, BIT_KISS_ACTIVE);
    }
}

/* Send the KISS exit frame. A trailing CR flushes it from the CAT parser in
 * case the radio was not in KISS after all. */
static void send_kiss_exit(void)
{
    radio_usb_write(KISS_EXIT, sizeof(KISS_EXIT), 500);
    vTaskDelay(pdMS_TO_TICKS(KISS_EXIT_MS));
    radio_usb_write((const uint8_t *)"\r", 1, 500);
    vTaskDelay(pdMS_TO_TICKS(100));
    cat_reset();
}

/*
 * Bring the radio's CAT port up: the TM-D750 only answers after a DTR rising
 * edge once it is ready. If it still does not answer it is probably stuck in
 * KISS from an earlier session (KISS survives DTR drops and replugs), so send
 * the KISS exit frame and keep trying.
 */
static bool wake_radio(void)
{
    char reply[CAT_LINE_MAX];
    for (int attempt = 1; attempt <= WAKE_ATTEMPTS && radio_usb_is_connected(); attempt++) {
        radio_usb_set_dtr(false);
        vTaskDelay(pdMS_TO_TICKS(DTR_LOW_MS));
        radio_usb_set_dtr(true);
        vTaskDelay(pdMS_TO_TICKS(DTR_HIGH_MS));
        cat_reset();
        if (cat_command("ID", reply, sizeof(reply), CAT_TIMEOUT_MS) == ESP_OK &&
            strncmp(reply, "ID ", 3) == 0) {
            ESP_LOGI(TAG, "radio answering (%s) after %d DTR pulse(s)", reply, attempt);
            return true;
        }
        ESP_LOGI(TAG, "no CAT reply; sending KISS exit in case the TNC is in KISS");
        send_kiss_exit();
    }
    return false;
}

/* ---------- state transitions (control task only) ---------- */

static void radio_bringup(void)
{
    s_radio_ok = false;
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    if (!wake_radio()) {
        ESP_LOGE(TAG, "radio does not answer CAT on USB; will retry");
        return;
    }

    char reply[CAT_LINE_MAX];
    if (cat_query("FQ 0", reply, sizeof(reply)) == ESP_OK) {
        ESP_LOGI(TAG, "band A: %s", reply);
    }

    char mode, band;
    if (cat_query("TN", reply, sizeof(reply)) != ESP_OK || !parse_tn(reply, &mode, &band)) {
        ESP_LOGE(TAG, "cannot read TNC mode (TN)");
        return;
    }
    char saved[sizeof(s_orig_tn)];
    if (nvs_load_orig(saved, sizeof(saved))) {
        /* A session was interrupted: the current mode is ours, not the user's. */
        strlcpy(s_orig_tn, saved, sizeof(s_orig_tn));
        ESP_LOGW(TAG, "recovering from an interrupted session; radio says %s, original %s",
                 reply, s_orig_tn);
    } else {
        strlcpy(s_orig_tn, reply, sizeof(s_orig_tn));
        ESP_LOGI(TAG, "radio TNC mode: %s", s_orig_tn);
    }
    s_in_kiss = false;
    s_radio_ok = true;
}

static bool enter_kiss(void)
{
    char mode, band;
    if (!parse_tn(s_orig_tn, &mode, &band)) {
        band = '1';
    }
    char cmd[16], reply[CAT_LINE_MAX];
    snprintf(cmd, sizeof(cmd), "TN 2,%c", band);

    nvs_store_orig(s_orig_tn); /* before switching, so a reset can undo it */
    if (cat_query(cmd, reply, sizeof(reply)) != ESP_OK || strcmp(reply, cmd) != 0) {
        ESP_LOGE(TAG, "radio refused %s", cmd);
        nvs_store_orig(NULL);
        return false;
    }
    s_in_kiss = true;
    s_dropped_to_app = 0;
    set_kiss_active(true);
    ESP_LOGI(TAG, "KISS on (band %c); bridging app <-> radio", band == '0' ? 'A' : 'B');
    return true;
}

/* Put the TNC back the way the user had it and forget the saved mode. */
static void restore_tnc(void)
{
    char reply[CAT_LINE_MAX];
    if (cat_query("TN", reply, sizeof(reply)) == ESP_OK && strcmp(reply, s_orig_tn) != 0) {
        if (cat_query(s_orig_tn, reply, sizeof(reply)) != ESP_OK ||
            strcmp(reply, s_orig_tn) != 0) {
            ESP_LOGE(TAG, "could not restore %s", s_orig_tn);
            return;
        }
    }
    nvs_store_orig(NULL);
    ESP_LOGI(TAG, "radio TNC mode restored: %s", s_orig_tn);
}

static char tnc_band(void)
{
    char mode, band;
    return parse_tn(s_orig_tn, &mode, &band) ? band : '1';
}

/* CAT mode only. Put the band back to its pre-QSY frequency and VFO/memory mode. */
static void qsy_restore_cat(void)
{
    if (!s_qsy_saved) {
        return;
    }
    char reply[CAT_LINE_MAX];
    if (cat_query(s_orig_fq, reply, sizeof(reply)) != ESP_OK || strcmp(reply, s_orig_fq) != 0) {
        ESP_LOGE(TAG, "QSY restore: %s failed (%s)", s_orig_fq, reply);
    }
    if (s_orig_vm[strlen(s_orig_vm) - 1] != '0') { /* was not on the VFO */
        if (cat_query(s_orig_vm, reply, sizeof(reply)) != ESP_OK || strcmp(reply, s_orig_vm) != 0) {
            ESP_LOGE(TAG, "QSY restore: %s failed (%s)", s_orig_vm, reply);
        }
    }
    ESP_LOGI(TAG, "QSY restored: %s, %s", s_orig_fq, s_orig_vm);
    s_qsy_saved = false;
}

/* CAT mode only. Switch the TNC band to VFO and tune it to `hz`. */
static bool qsy_set_cat(uint32_t hz)
{
    const char band = tnc_band();
    char cmd[32], reply[CAT_LINE_MAX];

    if (!s_qsy_saved) {
        snprintf(cmd, sizeof(cmd), "VM %c", band);
        if (cat_query(cmd, s_orig_vm, sizeof(s_orig_vm)) != ESP_OK ||
            strncmp(s_orig_vm, "VM ", 3) != 0) {
            ESP_LOGE(TAG, "QSY: cannot read %s (%s)", cmd, s_orig_vm);
            return false;
        }
    }
    /* FQ only works on a VFO. */
    snprintf(cmd, sizeof(cmd), "VM %c,0", band);
    if (cat_query(cmd, reply, sizeof(reply)) != ESP_OK || strcmp(reply, cmd) != 0) {
        ESP_LOGE(TAG, "QSY: %s refused (%s)", cmd, reply);
        return false;
    }
    if (!s_qsy_saved) {
        snprintf(cmd, sizeof(cmd), "FQ %c", band);
        if (cat_query(cmd, s_orig_fq, sizeof(s_orig_fq)) != ESP_OK ||
            strncmp(s_orig_fq, "FQ ", 3) != 0) {
            ESP_LOGE(TAG, "QSY: cannot read %s (%s)", cmd, s_orig_fq);
            return false;
        }
        s_qsy_saved = true;
        ESP_LOGI(TAG, "QSY: saved %s, %s", s_orig_vm, s_orig_fq);
    }
    snprintf(cmd, sizeof(cmd), "FQ %c,%010lu", band, (unsigned long)hz);
    if (cat_query(cmd, reply, sizeof(reply)) != ESP_OK || strcmp(reply, cmd) != 0) {
        ESP_LOGE(TAG, "QSY: %s refused (%s)", cmd, reply);
        return false;
    }
    ESP_LOGI(TAG, "QSY: band %c now %lu.%06lu MHz", band == '0' ? 'A' : 'B',
             (unsigned long)(hz / 1000000), (unsigned long)(hz % 1000000));
    return true;
}

/* Leave KISS, run a CAT step, and go back into KISS. Control task only. */
static void qsy_handle(bridge_event_t type, uint32_t hz)
{
    if (!s_usb_up || !s_radio_ok || !s_in_kiss) {
        ESP_LOGW(TAG, "QSY ignored: no active KISS session");
        return;
    }
    set_kiss_active(false);
    send_kiss_exit();
    s_in_kiss = false;
    if (!wake_radio()) {
        ESP_LOGE(TAG, "QSY: radio silent after KISS exit");
        s_radio_ok = false;
        return;
    }
    if (type == EV_QSY) {
        qsy_set_cat(hz);
    } else {
        qsy_restore_cat();
    }
    if (s_want_kiss) {
        enter_kiss();
    }
}

static void leave_kiss(void)
{
    set_kiss_active(false);
    send_kiss_exit();
    s_in_kiss = false;
    if (s_dropped_to_app) {
        ESP_LOGW(TAG, "%lu radio bytes dropped (app too slow)", (unsigned long)s_dropped_to_app);
    }
    if (!wake_radio()) {
        ESP_LOGE(TAG, "radio silent after KISS exit");
        s_radio_ok = false;
        return;
    }
    qsy_restore_cat();
    restore_tnc();
    ESP_LOGI(TAG, "KISS off");
}

static void reconcile(void)
{
    if (!s_usb_up || !s_radio_ok) {
        return;
    }
    if (s_want_kiss && !s_in_kiss) {
        enter_kiss();
    } else if (!s_want_kiss && s_in_kiss) {
        leave_kiss();
    } else if (!s_want_kiss && !s_in_kiss) {
        char saved[sizeof(s_orig_tn)];
        if (nvs_load_orig(saved, sizeof(saved))) {
            restore_tnc(); /* leftover from an interrupted session */
        }
    }
}

static void control_task(void *arg)
{
    (void)arg;
    for (;;) {
        bridge_msg_t msg;
        if (xQueueReceive(s_events, &msg, pdMS_TO_TICKS(RETRY_PERIOD_MS)) == pdTRUE) {
            switch (msg.type) {
            case EV_USB_UP:
                ESP_LOGI(TAG, "radio USB up");
                s_usb_up = true;
                radio_bringup();
                break;
            case EV_USB_DOWN:
                ESP_LOGI(TAG, "radio USB down");
                s_usb_up = false;
                s_radio_ok = false;
                s_in_kiss = false; /* radio may still be in KISS; NVS remembers */
                set_kiss_active(false);
                break;
            case EV_SESSION_OPEN:
                s_want_kiss = true;
                break;
            case EV_SESSION_CLOSE:
                s_want_kiss = false;
                break;
            case EV_QSY:
            case EV_QSY_RESTORE:
                qsy_handle(msg.type, msg.arg);
                xEventGroupSetBits(s_flags, BIT_QSY_DONE);
                break;
            }
        } else if (s_usb_up && !s_radio_ok) {
            radio_bringup();
        }
        reconcile();
    }
}

/* Writes app bytes to the radio once KISS is up. app_tx task only. */
static void forward_to_radio(const uint8_t *data, size_t len)
{
    EventBits_t bits = xEventGroupWaitBits(s_flags, BIT_KISS_ACTIVE, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(APP_TX_WAIT_MS));
    if (!(bits & BIT_KISS_ACTIVE)) {
        ESP_LOGW(TAG, "KISS not active; dropped %u bytes from app", (unsigned)len);
        return;
    }
    esp_err_t err = radio_usb_write(data, len, 2000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write to radio failed: %s", esp_err_to_name(err));
    }
}

#if CONFIG_KK_LINK_QSY_EXPERIMENT
/* Hand a CAT step to the control task and wait for it, so later frames are
 * not written while the radio is out of KISS. */
static void run_qsy_step(bridge_event_t type, uint32_t hz)
{
    xEventGroupClearBits(s_flags, BIT_QSY_DONE);
    if (post_arg(type, hz)) {
        xEventGroupWaitBits(s_flags, BIT_QSY_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(QSY_WAIT_MS));
    }
}

/* A SetHardware frame from the app: handle the ones we know, never pass to the radio. */
static void handle_sethw(const uint8_t *p, size_t n)
{
    if (n >= 5 && p[0] == SETHW_QSY) {
        uint32_t hz = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) |
                      ((uint32_t)p[3] << 8) | p[4];
        ESP_LOGI(TAG, "app QSY request: %lu Hz", (unsigned long)hz);
        run_qsy_step(EV_QSY, hz);
    } else if (n >= 1 && p[0] == SETHW_QSY_RESTORE) {
        ESP_LOGI(TAG, "app QSY restore request");
        run_qsy_step(EV_QSY_RESTORE, 0);
    } else {
        ESP_LOGI(TAG, "app SetHardware 0x%02x (%u bytes) ignored", n ? p[0] : 0, (unsigned)n);
    }
}

/* One complete frame, FEND to FEND. */
static void dispatch_frame(const uint8_t *f, size_t n)
{
    if (n >= 3 && (f[1] & 0x0F) == KISS_CMD_SETHW) {
        uint8_t payload[16];
        size_t len = 0;
        bool esc = false;
        for (size_t i = 2; i < n - 1 && len < sizeof(payload); i++) {
            uint8_t c = f[i];
            if (esc) {
                payload[len++] = c == KISS_TFEND ? KISS_FEND : c == KISS_TFESC ? KISS_FESC : c;
                esc = false;
            } else if (c == KISS_FESC) {
                esc = true;
            } else {
                payload[len++] = c;
            }
        }
        handle_sethw(payload, len);
        return;
    }
    forward_to_radio(f, n);
}

/* Reassemble KISS frames from the app's byte stream (frames may span writes). */
static void app_tx_task(void *arg)
{
    (void)arg;
    static uint8_t frame[KISS_FRAME_MAX];
    size_t flen = 0; /* frame[0] is the opening FEND when flen > 0 */
    uint8_t buf[256];
    for (;;) {
        size_t n = xStreamBufferReceive(s_app_tx, buf, sizeof(buf), portMAX_DELAY);
        for (size_t i = 0; i < n; i++) {
            const uint8_t c = buf[i];
            if (c == KISS_FEND) {
                if (flen > 1) {
                    frame[flen++] = KISS_FEND;
                    dispatch_frame(frame, flen);
                }
                frame[0] = KISS_FEND; /* closing FEND doubles as the next opening */
                flen = 1;
            } else if (flen == 0) {
                forward_to_radio(&c, 1); /* stray byte outside any frame */
            } else if (flen < sizeof(frame) - 1) {
                frame[flen++] = c;
            } else {
                ESP_LOGW(TAG, "KISS frame too long; passing through unparsed");
                forward_to_radio(frame, flen);
                flen = 0;
            }
        }
    }
}
#else
/* Moves app data to the radio once KISS is up. */
static void app_tx_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;) {
        size_t n = xStreamBufferReceive(s_app_tx, buf, sizeof(buf), portMAX_DELAY);
        if (n > 0) {
            forward_to_radio(buf, n);
        }
    }
}
#endif

esp_err_t bridge_start(const char *ble_name)
{
    s_events = xQueueCreate(16, sizeof(bridge_msg_t));
    s_flags = xEventGroupCreate();
    s_app_tx = xStreamBufferCreate(APP_TX_BUF_SIZE, 1);
    ESP_RETURN_ON_FALSE(s_events && s_flags && s_app_tx, ESP_ERR_NO_MEM, TAG, "no memory");
    ESP_RETURN_ON_ERROR(cat_init(), TAG, "cat_init");

    BaseType_t ok = xTaskCreate(control_task, "bridge", 4096, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "control task");
    ok = xTaskCreate(app_tx_task, "app_tx", 3072, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "app_tx task");

    const radio_usb_config_t usb_config = {
        .rx_cb = on_radio_rx,
        .state_cb = on_radio_state,
    };
    ESP_RETURN_ON_ERROR(radio_usb_start(&usb_config), TAG, "radio_usb_start");

    const ble_kiss_config_t ble_config = {
        .device_name = ble_name,
        .rx_cb = on_app_rx,
        .session_cb = on_app_session,
    };
    return ble_kiss_start(&ble_config);
}
