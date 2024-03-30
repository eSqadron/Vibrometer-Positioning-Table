/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
 */
#pragma once

typedef enum {
	SCAN_SUCCESS = 0,
        SCAN_DRIVER_ERROR = 1,

        SCAN_WRONG_STATUS = 2,

} scanner_return_codes_t;
