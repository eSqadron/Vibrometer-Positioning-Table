#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "scan_adc.h"
#include "scan_driver.h"
#include "scanner_return_codes.h"
#include "scan_buffer.h"

static struct ScannerDefinition new_scanner;

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
		      "New scanner Yaw axis defined succesfully!\n"
		      );

    return 0;
}

static int cmd_scanner_define_pitch(const struct shell *shell, size_t argc, char *argv[])
{
	struct ScannerAxis new_axis;
	parse_args_to_axis(argv, &new_axis);
	define_axis(new_axis, Pitch);

	shell_fprintf(shell, SHELL_NORMAL,
		      "New scanner Pitch axis defined succesfully!\n"
		      );

    return 0;
}

static int cmd_scanner_define_time(const struct shell *shell, size_t argc, char *argv[])
{
	new_scanner.wait_time = (unsigned int)strtol(argv[1], NULL, 10);

	shell_fprintf(shell, SHELL_NORMAL,
		      "New scanner wait ime between points specified to %d!\n",
		      new_scanner.wait_time
		      );

	return 0;
}

static int cmd_scanner_ready(const struct shell *shell, size_t argc, char *argv[])
{
	scanner_return_codes_t scan_ret;
	scan_ret = define_scanner(new_scanner);

	switch (scan_ret) {
	case SCAN_SUCCESS:
		shell_fprintf(shell, SHELL_NORMAL, "Sucessfully defined scanner!\n");
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
	scanner_return_codes_t scan_ret;
	scan_ret = start_scanner();

	switch (scan_ret) {
	case SCAN_SUCCESS:
		shell_fprintf(shell, SHELL_NORMAL, "Sucessfully started scan!\n");
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
	shell_fprintf(shell, SHELL_NORMAL, "status: %d\n", ret_status);
	return 0;
}

static int cmd_scanner_measure(const struct shell *shell, size_t argc, char *argv[])
{
	int value;
	scanner_return_codes_t ret;
	ret = perform_meas(&value);
	if(ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Error while measuring! - %d\n", ret);
	} else {
		shell_fprintf(shell, SHELL_NORMAL, "Measured value: %d\n", value);
	}

	return 0;
}

static int cmd_scanner_dump(const struct shell *shell, size_t argc, char *argv[])
{
	unsigned int buff_size;
	scanner_return_codes_t ret;

	struct ScanPoint *buff;

	buff_size = get_buffer_size();

	if (buff_size == 0) {
		shell_fprintf(shell, SHELL_NORMAL, "Buffer is empty\n");
	}

	ret = get_buffer(&buff);
	if(ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Couldn't get buffer! - %d\n", ret);
	}

	for(unsigned int i = 0; i < buff_size; ++i) {
		shell_fprintf(shell, SHELL_NORMAL, "Yaw: %d, Pitch: %d, Value: %d\n",
			      buff[i].yaw,
			      buff[i].pitch,
			      buff[i].meas_value);
	}

	ret = clear_buffer();
	if(ret != SCAN_SUCCESS) {
		shell_fprintf(shell, SHELL_NORMAL, "Couldn't clear buffer! - %d\n", ret);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scanner_define,
	SHELL_CMD_ARG(yaw, NULL, "Define yaw scanning axis.\nArgs: <channel> <start deg> <stop deg> <delta deg>", cmd_scanner_define_yaw, 5, 0),
	SHELL_CMD_ARG(pitch, NULL, "Define pitch scanning axis.\nArgs: <channel> <start deg> <stop deg> <delta deg>", cmd_scanner_define_pitch, 5, 0),
	SHELL_CMD_ARG(time, NULL, "Define wait time between moving and scanning (in msec)", cmd_scanner_define_time, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scanner,
	SHELL_CMD(define, &sub_scanner_define, "Active template", NULL),
	SHELL_CMD(ready, NULL, "Check scanner readiness and initialise prepared scanner", cmd_scanner_ready),
	SHELL_CMD(start, NULL, "Start Scanner", cmd_scanner_start),
	SHELL_CMD(status, NULL, "Get current status", cmd_scanner_get_status),
	SHELL_CMD(measure, NULL,
		  "Perform one time measurement,outside of normal operation",
		  cmd_scanner_measure),
	SHELL_CMD(dump, NULL,
		  "Dump measured points",
		  cmd_scanner_dump),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(scan, &sub_scanner, "Scanner", NULL);
