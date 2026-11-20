/*
 * SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Jon Schulman
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * kk-link milestone 1: USB host spike.
 *
 * Waits for a Kenwood TM-D750 on the ESP32-S3's native USB port, then runs a
 * few read-only CAT queries and logs the replies. Unplugging and replugging
 * the radio re-opens the port and repeats the queries.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cat.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "radio_usb.h"

static const char *TAG = "kk_link";

#define CAT_TIMEOUT_MS   500
#define CAT_ATTEMPTS     3
#define WAKE_ATTEMPTS    20  /* DTR toggles before giving up on a connection */
#define SETTLE_MS        500 /* after open, before the first DTR rising edge */
#define DTR_LOW_MS       250
#define DTR_HIGH_MS      250 /* after the rising edge, before sending */

static QueueHandle_t s_link_events; /* bool: true = connected */

static void on_radio_rx(const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    cat_feed(data, len);
}

static void on_radio_state(bool connected, void *ctx)
{
    (void)ctx;
    xQueueOverwrite(s_link_events, &connected);
}

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

/*
 * The TM-D750 only serves CAT on USB after it sees DTR rise once it is ready,
 * so pulse DTR low -> high and probe with ID until it answers.
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
            reply_matches("ID", reply)) {
            ESP_LOGI(TAG, "radio answering after %d DTR pulse(s)", attempt);
            return true;
        }
    }
    return false;
}

static void query(const char *cmd)
{
    char reply[CAT_LINE_MAX];
    esp_err_t err = cat_query(cmd, reply, sizeof(reply));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s -> %s", cmd, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "%s -> %s", cmd, reply);

    /* "FQ 0,0145470000": frequency in Hz after the comma. */
    if (strncmp(reply, "FQ ", 3) == 0) {
        const char *comma = strchr(reply, ',');
        if (comma) {
            unsigned long long hz = strtoull(comma + 1, NULL, 10);
            ESP_LOGI(TAG, "band %c frequency %llu.%06llu MHz", reply[3], hz / 1000000ULL,
                     hz % 1000000ULL);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "kk-link USB host spike starting");
    /* For descriptor dumps and hex dumps of radio RX, uncomment: */
    /* esp_log_level_set("radio_usb", ESP_LOG_DEBUG); */

    s_link_events = xQueueCreate(1, sizeof(bool));
    ESP_ERROR_CHECK(s_link_events ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(cat_init());

    const radio_usb_config_t usb_config = {
        .rx_cb = on_radio_rx,
        .state_cb = on_radio_state,
        .ctx = NULL,
    };
    ESP_ERROR_CHECK(radio_usb_start(&usb_config));

    for (;;) {
        bool connected;
        xQueueReceive(s_link_events, &connected, portMAX_DELAY);
        if (!connected) {
            ESP_LOGI(TAG, "radio gone; waiting for it to come back");
            continue;
        }

        ESP_LOGI(TAG, "radio connected; raising DTR and querying (read-only)");
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
        if (!wake_radio()) {
            ESP_LOGE(TAG, "radio does not answer CAT on USB after %d DTR pulses; "
                          "replug to retry", WAKE_ATTEMPTS);
            continue;
        }
        query("ID");
        query("FQ 0");
    }
}
