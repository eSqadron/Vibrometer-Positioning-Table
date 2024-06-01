/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Jakub Mazur
 *
 */
#pragma once
#include "scanner_return_codes.h"

struct ScanPoint {
	int yaw;
	int pitch;
#if defined(CONFIG_AUTO_MEASUREMENTS)
	int meas_value;
#endif
};

scanner_return_codes_t add_point(struct ScanPoint new_point);

unsigned int get_buffer_size(void);

scanner_return_codes_t get_buffer(struct ScanPoint **first_point_out);

scanner_return_codes_t clear_buffer(void);
