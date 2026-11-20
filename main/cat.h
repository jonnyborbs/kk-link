/*
 * SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Jon Schulman
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * cat: line-oriented CAT command/response on top of radio_usb.
 *
 * The TM-D750 answers CAT commands with one CR-terminated line ("ID TM-D750",
 * "FQ 0,0145470000", or "?" for an unknown/bad command).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAT_LINE_MAX 128

/* Create the line queue. Call before radio_usb_start(). */
esp_err_t cat_init(void);

/* Feed raw bytes from the radio (radio_usb rx callback). */
void cat_feed(const uint8_t *data, size_t len);

/* Drop any partial line and queued lines (e.g. after reconnect). */
void cat_reset(void);

/*
 * Send `cmd` followed by CR and wait up to `timeout_ms` for one reply line
 * (without the terminator). Returns ESP_ERR_TIMEOUT if nothing came back.
 */
esp_err_t cat_command(const char *cmd, char *reply, size_t reply_size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
