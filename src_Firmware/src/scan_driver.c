#include <zephyr/kernel.h>

#include <stdlib.h>

#include "scan_driver.h"
#include "scanner_return_codes.h"
#include "scan_adc.h"
#include "scan_buffer.h"

// TODO - error handling, both from driver and proper returns in this functions!

#define TIME_BETWEEN_POS_CHECKS_MSEC 1000
#define SCANNER_AXES 2

static struct ScannerDefinition scanner = {
	.status = Uninitialised
};

static enum MotorDirection scanning_direction = FORWARD;

struct k_timer pause_timer;
struct k_timer wait_for_point_timer;

static int target[SCANNER_AXES];

static bool if_taget_achieved(uint32_t target_pos, uint32_t current_position)
{
	// TODO - add possibility to return actual max position from driver!
	return abs(current_position-target_pos) <=
		(360u * CONFIG_POSITION_CONTROL_MODIFIER) /
		(CONFIG_POS_CONTROL_PRECISION_MODIFIER);
}

static scanner_return_codes_t go_to_point()
{
	return_codes_t ret;
	// TODO - this could be moved to megaturbomacro to "primary" motor driver
	// printk("Go To point %d, %d\n", target[Yaw], target[Pitch]);
	uint32_t pos_value;

	for (int i = 0; i < SCANNER_AXES; ++i) {
		ret = position_get(&pos_value, scanner.axes[i].channel);
		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}

		if(if_taget_achieved(target[i], pos_value)) {
			continue;
		}
		// TODO - uncomment when pwm HW bug is fixed
		// for now, two channels shouldn't be started at the same time!
		// if (!get_motor_off_on(scanner.axes[i].channel)) {
		// 	motor_on(FORWARD, scanner.axes[i].channel);
		// }

		ret = target_position_set(target[i], scanner.axes[i].channel);

		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}
	}

	k_timer_start(&wait_for_point_timer, K_MSEC(TIME_BETWEEN_POS_CHECKS_MSEC), K_NO_WAIT);
	return SCAN_SUCCESS;
}

static void wait_for_point_handler(struct  k_work *dummy)
{
	return_codes_t ret;
	uint32_t pos_value;
	bool tagets_acheved = true; // Target achieved on BOTH channels!

	for (int i = 0; i < SCANNER_AXES; ++i) {
		ret = position_get(&pos_value, scanner.axes[i].channel);
		if (ret != SUCCESS) {
			scanner.status = Error;
			return;
		}

		// printk("CH: %d, v: %d\n", scanner.axes[i].channel, pos_value);

		if (!if_taget_achieved(target[i], pos_value)) {
			// Target NOT Achieved on one of the channels!

			if (!get_motor_off_on(scanner.axes[i].channel)) {
				ret = motor_on(FORWARD, scanner.axes[i].channel);
				if (ret != SUCCESS) {
					scanner.status = Error;
					return;
				}
			}

			tagets_acheved = false;
			break; // Start only one channel at the time (due to pwm bug)
			// when this channel hits the target, if will be skipped and next channel
			// will start
		} else {
			ret = motor_off(scanner.axes[i].channel);
			if (ret != SUCCESS) {
				scanner.status = Error;
				return;
			}
		}
	}

	if (tagets_acheved) {
		// Point is achieved
		// printk("Point Achieved!\n");
		for (int i = 0; i < SCANNER_AXES; ++i) {
			ret = motor_off(scanner.axes[i].channel);
			if (ret != SUCCESS) {
				scanner.status = Error;
				return;
			}
		}
		k_timer_start(&pause_timer, K_MSEC(scanner.wait_time), K_NO_WAIT);
		return;
	} else {
		k_timer_start(&wait_for_point_timer,
			      K_MSEC(TIME_BETWEEN_POS_CHECKS_MSEC),
			      K_NO_WAIT);
		return;
	}
}

static scanner_return_codes_t finish_scan(void)
{
	scanner.status = Finished;
	// TODO, disable motors, etc.

	return SCAN_DRIVER_ERROR;
}

static void pause_timer_handler(struct  k_work *dummy)
{
	int out_val;
	scanner_return_codes_t scan_ret;
	return_codes_t ret;
	scan_ret = perform_meas(&out_val);
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}

	uint32_t pos_value;
	struct ScanPoint new_point;
	ret = position_get(&pos_value, scanner.axes[Yaw].channel);
	if (ret != SUCCESS) {
		scanner.status = Error;
		return;
	}

	new_point.yaw = pos_value;
	ret = position_get(&pos_value, scanner.axes[Pitch].channel);
	if (ret != SUCCESS) {
		scanner.status = Error;
		return;
	}

	new_point.pitch = pos_value;

	new_point.meas_value = out_val;

	scan_ret = add_point(new_point);
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}

	// TODO - Q&A about measurement? Multiple measurements with wait time in between?
	// TODO - maybe turn off motor or lock it?

	if (scanning_direction == FORWARD) {
		target[Yaw] += scanner.axes[Yaw].delta;
	} else if (scanning_direction == BACKWARD) {
		target[Yaw] -= scanner.axes[Yaw].delta;
	}

	// If row was ended
	if ((target[Yaw] > scanner.axes[Yaw].end && scanning_direction == FORWARD) ||
	    (target[Yaw] < scanner.axes[Yaw].start && scanning_direction == BACKWARD)) {

		// undo last step
		if (scanning_direction == FORWARD) {
			target[Yaw] -= scanner.axes[Yaw].delta;
		} else if (scanning_direction == BACKWARD) {
			target[Yaw] += scanner.axes[Yaw].delta;
		}

		// move to next row
		target[Pitch] += scanner.axes[Pitch].delta;

		// change scanning direction
		if (scanning_direction == FORWARD) {
			scanning_direction = BACKWARD;
		} else if (scanning_direction == BACKWARD) {
			scanning_direction = FORWARD;
		}
	}

	if (target[Pitch] > scanner.axes[Pitch].end) {
		// scanning is finished!
		// printk("Finished!\n");
		scan_ret = finish_scan();
		if (scan_ret != SCAN_SUCCESS) {
			scanner.status = Error;
			return;
		}

		return;
	}

	scan_ret = go_to_point();
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}
}

K_WORK_DEFINE(pause_timer_work, pause_timer_handler);
static void pause_timer_handler_wrapper(struct k_timer *dummy)
{
	k_work_submit(&pause_timer_work);
}

K_TIMER_DEFINE(pause_timer, pause_timer_handler_wrapper, NULL);


K_WORK_DEFINE(wait_for_point_work, wait_for_point_handler);
static void wait_for_point_wrapper(struct k_timer *dummy)
{
	k_work_submit(&wait_for_point_work);
}

K_TIMER_DEFINE(wait_for_point_timer, wait_for_point_wrapper, NULL);

scanner_return_codes_t define_scanner(struct ScannerDefinition new_scanner)
{
	return_codes_t ret;
	if (scanner.status == Ready || scanner.status == Uninitialised) {
		scanner = new_scanner;

		// prepare driver
		ret = mode_set(POSITION);
		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}
		ret = motor_off(CH0);
		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}
		ret = motor_off(CH1);
		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}

		scanner.status = Ready;

		return SCAN_SUCCESS;
	}

	return SCAN_WRONG_STATUS;
}

scanner_return_codes_t start_scanner(void)
{
	scanner_return_codes_t scan_ret;
	if (scanner.status == Ready) {
		// printk("Starting!");
		target[Yaw] = scanner.axes[Yaw].start;
		target[Pitch] = scanner.axes[Pitch].start;
		scan_ret = go_to_point();

		if(scan_ret != SCAN_SUCCESS) {
			return scan_ret;
		}

		scanner.status = Scanning;

		return SCAN_SUCCESS;
	}

	return SCAN_WRONG_STATUS;
}

enum ScannerStatus get_status(void) {
	return scanner.status;
}
