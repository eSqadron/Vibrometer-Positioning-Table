/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Maciej Baczmanski, Michal Kawiak, Jakub Mazur
 */
#pragma once

typedef enum {
	SCAN_SUCCESS = 0,
	SCAN_DRIVER_ERROR = 1,

	SCAN_WRONG_STATUS = 2,

	ADC_READ_FAIL = 3,
	ADC_CONVERSION_FAIL = 4,
	ADC_NOT_INITIALISED = 5,

	BUFF_EMPTY = 6,

} scan_return_codes_t;
