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

static const uint8_t KISS_EXIT[] = { 0xC0, 0xFF, 0xC0 };

typedef enum {
    EV_USB_UP,
    EV_USB_DOWN,
    EV_SESSION_OPEN,
    EV_SESSION_CLOSE,
} bridge_event_t;

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

/* ---------- callbacks (other tasks' context) ---------- */

static void post(bridge_event_t ev)
{
    if (xQueueSend(s_events, &ev, 0) != pdTRUE) {
        ESP_LOGE(TAG, "event queue full, lost event %d", ev);
    }
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
        bridge_event_t ev;
        if (xQueueReceive(s_events, &ev, pdMS_TO_TICKS(RETRY_PERIOD_MS)) == pdTRUE) {
            switch (ev) {
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
            }
        } else if (s_usb_up && !s_radio_ok) {
            radio_bringup();
        }
        reconcile();
    }
}

/* Moves app data to the radio once KISS is up. */
static void app_tx_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;) {
        size_t n = xStreamBufferReceive(s_app_tx, buf, sizeof(buf), portMAX_DELAY);
        if (n == 0) {
            continue;
        }
        EventBits_t bits = xEventGroupWaitBits(s_flags, BIT_KISS_ACTIVE, pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(APP_TX_WAIT_MS));
        if (!(bits & BIT_KISS_ACTIVE)) {
            ESP_LOGW(TAG, "KISS not active; dropped %u bytes from app", (unsigned)n);
            continue;
        }
        esp_err_t err = radio_usb_write(buf, n, 2000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "write to radio failed: %s", esp_err_to_name(err));
        }
    }
}

esp_err_t bridge_start(const char *ble_name)
{
    s_events = xQueueCreate(16, sizeof(bridge_event_t));
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
