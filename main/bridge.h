/*
 * SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Jon Schulman
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * bridge: ties the BLE-KISS service to the radio's USB serial port.
 *
 * While no app is subscribed the radio port is in CAT mode. When an app
 * subscribes, the radio's TNC is switched to KISS (CAT "TN 2,<band>") and
 * bytes are passed through in both directions. When the app leaves, KISS is
 * exited with the KISS frame C0 FF C0 and the radio's original TNC mode is
 * restored. The original mode is kept in NVS so it is also restored after a
 * reset or cable drop in the middle of a session.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start USB host, BLE and the bridge tasks. NVS must be initialised. */
esp_err_t bridge_start(const char *ble_name);

#ifdef __cplusplus
}
#endif
