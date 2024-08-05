/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Jakub Mazur
 *
 */
#include "driver.h"
#include "scanner_return_codes.h"
#include "scan_buffer.h"

struct ScannerAxis {
	int32_t start;
	int32_t end;
	uint32_t delta;
	enum ChannelNumber channel;
};

enum ScannerAxes {
	Yaw,
	Pitch
};

enum ScannerStatus {
	Uninitialised,
	Ready,
	Scanning,
#if !defined(CONFIG_AUTO_MEASUREMENTS)
	WaitingForContinuation,
#endif
	Stopping,
	Finished,
	Error,
};

// source:
// https://www.linkedin.com/pulse/mapping-enum-string-c-language-sathishkumar-duraisamy/
static const char * const status_names[] = {
	[Uninitialised] = "Uninitialised",
	[Ready] = "Ready",
	[Scanning] = "Scanning",
	[WaitingForContinuation] = "WaitingForContinuation",
	[Stopping] = "Stopping",
	[Finished] = "Finished",
	[Error] = "Error",
};

struct ScannerDefinition {
	struct ScannerAxis axes[2];
	enum ScannerStatus status;
#if defined(CONFIG_AUTO_MEASUREMENTS)
	unsigned int wait_time;
#endif
};

scan_return_codes_t get_current_point(struct ScanPoint *new_point);

scan_return_codes_t move_to_next_point(void);

scan_return_codes_t define_scanner(struct ScannerDefinition new_scanner);

scan_return_codes_t start_scanner(void);

enum ScannerStatus get_status(void);

scan_return_codes_t reset_scanner(void);

scan_return_codes_t stop_scanner(void);

char *get_status_as_string(enum ScannerStatus status);

struct DriverVersion get_scanner_version(void);
