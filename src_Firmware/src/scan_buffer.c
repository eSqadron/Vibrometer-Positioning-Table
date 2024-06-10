// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Jakub Mazur
#include <zephyr/kernel.h>
#include <stdlib.h>

# include "scan_buffer.h"

static unsigned int buffer_size; // 0 by default

static struct ScanPoint *scan_buffer;

scan_return_codes_t add_point(struct ScanPoint new_point)
{
	if (buffer_size == 0) {
		scan_buffer = malloc(1 * sizeof(struct ScanPoint));
		scan_buffer[0] = new_point;
		buffer_size = 1;
	} else {
		buffer_size += 1;
		scan_buffer = realloc(scan_buffer, buffer_size * sizeof(struct ScanPoint));
		scan_buffer[buffer_size-1] = new_point;
	}

	return SCAN_SUCCESS;
}

unsigned int get_buffer_size(void)
{
	return buffer_size;
}

scan_return_codes_t get_buffer(struct ScanPoint **first_point_out)
{
	if (buffer_size == 0) {
		return BUFF_EMPTY;
	}

	*first_point_out = scan_buffer;

	return SCAN_SUCCESS;
}

scan_return_codes_t clear_buffer(void)
{
	if (buffer_size == 0) {
		return BUFF_EMPTY;
	}
	buffer_size = 0;
	free(scan_buffer);

	return SCAN_SUCCESS;
}
