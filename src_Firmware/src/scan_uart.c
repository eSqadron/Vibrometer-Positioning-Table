// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Jakub Mazur
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "scan_adc.h"
#include "scan_driver.h"
#include "scanner_return_codes.h"
#include "scan_buffer.h"

static struct ScannerDefinition new_scanner;

struct DefineStatus {
	bool Pitch;
	bool Yaw;
#if defined(CONFIG_AUTO_MEASUREMENTS)
	bool Time;
#endif
};

static struct DefineStatus define_status = {
	false,
	false,
#if defined(CONFIG_AUTO_MEASUREMENTS)
	false
#endif
};

static int define_axis(struct ScannerAxis new_axis, enum ScannerAxes axis)
{
	new_scanner.axes[axis] = new_axis;

	return 0;
}

static int parse_args_to_axis(char *argv[], struct ScannerAxis *out_axis)
{
	out_axis->channel = (enum ChannelNumber)strtol(argv[1], NULL, 10);
	out_axis->start = (int)strtol(argv[2], NULL, 10);
	out_axis->end = (int)strtol(argv[3], NULL, 10);
	out_axis->delta = (unsigned int)strtol(argv[4], NULL, 10);

	return 0;
}

static int cmd_scanner_define_yaw(const struct shell *shell, size_t argc, char *argv[])
{
	struct ScannerAxis new_axis;

	parse_args_to_axis(argv, &new_axis);
	define_axis(new_axis, Yaw);

	shell_fprintf(shell, SHELL_NORMAL,
		      "New scanner Yaw axis defined successfully!\n"
		      );

	define_status.Yaw = true;

	return 0;
}

static int cmd_scanner_define_pitch(const struct shell *shell, size_t argc, char *argv[])
{
	struct ScannerAxis new_axis;

	parse_args_to_axis(argv, &new_axis);
	define_axis(new_axis, Pitch);

	shell_fprintf(shell, SHELL_NORMAL,
		      "New scanner Pitch axis defined successfully!\n"
		      );
	define_status.Pitch = true;

	return 0;
}

#if defined(CONFIG_AUTO_MEASUREMENTS)
static int cmd_scanner_define_time(const struct shell *shell, size_t argc, char *argv[])
{
	new_scanner.wait_time = (unsigned int)strtol(argv[1], NULL, 10);

	shell_fprintf(shell, SHELL_NORMAL,
		      "New scanner wait ime between points specified to %d!\n",
		      new_scanner.wait_time
		      );

	define_status.Time = true;

	return 0;
}
#endif

static int cmd_scanner_ready(const struct shell *shell, size_t argc, char *argv[])
{
	if (!define_status.Yaw) {
		shell_fprintf(shell, SHELL_ERROR, "Yaw is undefined!\n");
		return 0;
	}

	if (!define_status.Pitch) {
		shell_fprintf(shell, SHELL_ERROR, "Pitch is undefined!\n");
		return 0;
	}

#if defined(CONFIG_AUTO_MEASUREMENTS)
	if (!define_status.Time) {

		shell_fprintf(shell, SHELL_ERROR, "Delta Time is undefined!\n");
		return 0;
	}
#endif

	scan_return_codes_t scan_ret;

	scan_ret = define_scanner(new_scanner);

	switch (scan_ret) {
	case SCAN_SUCCESS:
		shell_fprintf(shell, SHELL_NORMAL, "Successfully defined scanner!\n");
		// TODO - write exact scanner sepcification
	return 0;
	case SCAN_WRONG_STATUS:
		shell_fprintf(shell, SHELL_ERROR, "Scanner is in wrong status!\n");
	return 0;
	default:
		shell_fprintf(shell, SHELL_ERROR, "Other error - %d\n", scan_ret);
	return 0;
	}
}

static int cmd_scanner_start(const struct shell *shell, size_t argc, char *argv[])
{
	scan_return_codes_t scan_ret;

	scan_ret = start_scanner();

	switch (scan_ret) {
	case SCAN_SUCCESS:
		shell_fprintf(shell, SHELL_NORMAL, "Successfully started scan!\n");
	return 0;
	case SCAN_WRONG_STATUS:
		shell_fprintf(shell, SHELL_ERROR, "Scanner is in wrong status!\n");
	return 0;
	default:
		shell_fprintf(shell, SHELL_ERROR, "Other error - %d\n", scan_ret);
	return 0;
	}
}

static int cmd_scanner_get_status(const struct shell *shell, size_t argc, char *argv[])
{
	enum ScannerStatus ret_status = get_status();

	shell_fprintf(shell, SHELL_NORMAL, "status: %s\n", status_names[ret_status]);
	return 0;
}

static int cmd_scanner_dump(const struct shell *shell, size_t argc, char *argv[])
{
	unsigned int buff_size;
	scan_return_codes_t ret;

	struct ScanPoint *buff;

	buff_size = get_buffer_size();

	if (buff_size == 0) {
		shell_fprintf(shell, SHELL_NORMAL, "Buffer is empty!\n");
	} else {
		ret = get_buffer(&buff);
		if (ret != SCAN_SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "Couldn't get buffer! - %d\n", ret);
			return 0;
		}

		shell_fprintf(shell, SHELL_NORMAL, "Dumping last %d points!\n", buff_size);

		for (unsigned int i = 0; i < buff_size; ++i) {
#if defined(CONFIG_AUTO_MEASUREMENTS)
			shell_fprintf(shell, SHELL_NORMAL, "Yaw: %d, Pitch: %d, Value: %d\n",
				      buff[i].yaw,
				      buff[i].pitch,
				      buff[i].meas_value);
#endif
#if !defined(CONFIG_AUTO_MEASUREMENTS)
			shell_fprintf(shell, SHELL_NORMAL, "Yaw: %d, Pitch: %d\n",
				      buff[i].yaw,
				      buff[i].pitch);
#endif
		}

		ret = clear_buffer();
		if (ret != SCAN_SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL, "Couldn't clear buffer! - %d\n", ret);
			return 0;
		}
	}

	if (get_status() == Finished) {
		ret = reset_scanner();
		if (ret != SCAN_SUCCESS) {
			shell_fprintf(shell, SHELL_NORMAL,
				      "Couldn't reset scanner status! - %d\n", ret);
			return 0;
		}
	}

	return 0;
}


static int cmd_get_point(const struct shell *shell, size_t argc, char *argv[])
{
	scan_return_codes_t ret;
	struct ScanPoint new_point;

	ret = get_current_point(&new_point);

	if (ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Couldn't get buffer! - %d\n", ret);
		return 0;
	}

	shell_fprintf(shell, SHELL_NORMAL, "Yaw: %d, Pitch: %d\n",
			new_point.yaw,
			new_point.pitch);

	return 0;
}

static int cmd_next_point(const struct shell *shell, size_t argc, char *argv[])
{
	scan_return_codes_t ret;

	ret = move_to_next_point();

	if (ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Couldn't move to next point! - %d\n", ret);
		return 0;
	}

	shell_fprintf(shell, SHELL_NORMAL, "successfully started movement to next point!\n");

	return 0;
}

static int cmd_scanner_stop(const struct shell *shell, size_t argc, char *argv[])
{
	scan_return_codes_t ret;

	ret = stop_scanner();

	if (ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Couldn't stop the scanner! - %d\n", ret);
		return 0;
	}

	shell_fprintf(shell, SHELL_NORMAL, "successfully stopped scanner!\n");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scanner_define,
	SHELL_CMD_ARG(yaw, NULL,
		      "Define yaw scanning axis.\n"
		      "Args: <HW channel> <start deg> <stop deg> <delta deg>",
		      cmd_scanner_define_yaw, 5, 0),
	SHELL_CMD_ARG(pitch, NULL,
		      "Define pitch scanning axis.\n"
		      "Args: <HW channel> <start deg> <stop deg> <delta deg>",
		      cmd_scanner_define_pitch, 5, 0),
#if defined(CONFIG_AUTO_MEASUREMENTS)
	SHELL_CMD_ARG(time, NULL,
		      "Define wait time between moving and scanning (in msec)",
		      cmd_scanner_define_time, 2, 0),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scanner,
	SHELL_CMD(define, &sub_scanner_define, "Define scaner parameters", NULL),
	SHELL_CMD(ready, NULL,
		  "Check scanner readiness and initialise prepared scanner", cmd_scanner_ready),
	SHELL_CMD(start, NULL, "Start Scanner", cmd_scanner_start),
	SHELL_CMD(status, NULL, "Get current status", cmd_scanner_get_status),
	SHELL_CMD(get_point, NULL,
		  "Get current point and perform measurement outside of normal operation",
		  cmd_get_point),
#if !defined(CONFIG_AUTO_MEASUREMENTS)
	SHELL_CMD(next_point, NULL, "Continue to the next point", cmd_next_point),
#endif
	SHELL_CMD(dump, NULL,
		  "Dump points measured from previous dump (or measuement beginning), "
		  "clear existing buffer, and if measurement is finished, reset scanner to "
		  "ready status",
		  cmd_scanner_dump),
	SHELL_CMD(stop, NULL,
		  "Finish scanning prematurely",
		  cmd_scanner_stop),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(scan, &sub_scanner, "Scanner commands:", NULL);
