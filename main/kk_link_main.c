/*
 * SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Jon Schulman
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * kk-link: BLE-KISS adapter for the Kenwood TM-D750.
 *
 * iOS/Android KISS apps connect over BLE (BLE-KISS API); the adapter talks to
 * the radio over its main-unit USB port (CDC-ACM) as a USB host.
 */
#include "bridge.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "kk_link";

/* Placeholder name. Note: RadioMail only enables frequency control (QSY) for
 * adapters named "B.B. Link"; do not use that name without coordinating with
 * Island Magic. */
#define BLE_DEVICE_NAME "KK Link"

void app_main(void)
{
    ESP_LOGI(TAG, "kk-link starting");
    /* For descriptor dumps and hex dumps of radio RX, uncomment: */
    /* esp_log_level_set("radio_usb", ESP_LOG_DEBUG); */

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(bridge_start(BLE_DEVICE_NAME));
}
