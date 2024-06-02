// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Jakub Mazur
#include <zephyr/kernel.h>

#include <stdlib.h>

#include "scan_driver.h"
#include "scanner_return_codes.h"
#include "scan_adc.h"

// TODO - error handling, both from driver and proper returns in this functions!

#define SCANNER_AXES 2

static struct ScannerDefinition scanner = {
	.status = Uninitialised
};

static enum MotorDirection scanning_direction = FORWARD;

#if defined(CONFIG_AUTO_MEASUREMENTS)
struct k_timer pause_timer;
#endif
struct k_timer wait_for_point_timer;

#pragma region work_declarations
static void point_achieved_handler(struct  k_work *dummy);
static void wait_for_point_handler(struct  k_work *dummy);
K_WORK_DEFINE(point_achieved_work, point_achieved_handler);
K_WORK_DEFINE(wait_for_point_work, wait_for_point_handler);
#pragma endregion

static int target[SCANNER_AXES];

static scanner_return_codes_t finish_scan(void)
{
	scanner.status = Finished;
	// TODO, disable motors, etc.

	return SCAN_SUCCESS;
}

static scanner_return_codes_t go_to_point(void)
{
	return_codes_t ret;
	// TODO - this could be moved to megaturbomacro to "primary" motor driver
	// printk("Go To point %d, %d\n", target[Yaw], target[Pitch]);

	for (int i = 0; i < SCANNER_AXES; ++i) {
		ret = target_position_set(target[i], scanner.axes[i].channel);

		if (ret != SUCCESS) {
			return SCAN_DRIVER_ERROR;
		}
	}

	k_timer_start(&wait_for_point_timer, K_MSEC(CONFIG_TIME_BTWN_POS_CHECKS_MSEC), K_NO_WAIT);
	return SCAN_SUCCESS;
}

static void wait_for_point_handler(struct  k_work *dummy)
{
	return_codes_t ret;
	scanner_return_codes_t scan_ret;
	bool tagets_achieved = true; // Target achieved on BOTH channels!

	// If someone decided to break the scan prematurely:
	if (scanner.status == Stopping) {
		scan_ret = finish_scan();
		if (scan_ret != SCAN_SUCCESS) {
			scanner.status = Error;
			return;
		}
		return;
	}

	for (int i = 0; i < SCANNER_AXES; ++i) {
		// printk("CH: %d, v: %d\n", scanner.axes[i].channel, pos_value);

		if (!is_target_achieved(scanner.axes[i].channel)) {
			// Target NOT Achieved on one of the channels!

			if (!get_motor_off_on(scanner.axes[i].channel)) {
				ret = motor_on(FORWARD, scanner.axes[i].channel);
				if (ret != SUCCESS) {
					scanner.status = Error;
					return;
				}
			}

			tagets_achieved = false;
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

	if (tagets_achieved) {
		// Point is achieved - previous loop was finished without setting
		// tagets_acheved to false!
		// printk("Point Achieved!\n");
		for (int i = 0; i < SCANNER_AXES; ++i) {
			ret = motor_off(scanner.axes[i].channel);
			if (ret != SUCCESS) {
				scanner.status = Error;
				return;
			}
		}
#if defined(CONFIG_AUTO_MEASUREMENTS)
		k_timer_start(&pause_timer, K_MSEC(scanner.wait_time), K_NO_WAIT);
#else
		// Start function that handles situation, when target is achieved!
		k_work_submit(&point_achieved_work);
#endif
		return;
	}

	// if target is still not achieved, start timer that will run this function again!
	k_timer_start(&wait_for_point_timer,
			K_MSEC(CONFIG_TIME_BTWN_POS_CHECKS_MSEC),
			K_NO_WAIT);
}

static void point_achieved_handler(struct  k_work *dummy)
{
	scanner_return_codes_t scan_ret;
	struct ScanPoint new_point;

	if (scanner.status == Stopping) {
		scan_ret = finish_scan();
		if (scan_ret != SCAN_SUCCESS) {
			scanner.status = Error;
			return;
		}
		return;
	}

	scan_ret = get_current_point(&new_point);

	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}

	scan_ret = add_point(new_point);
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}

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

#if defined(CONFIG_AUTO_MEASUREMENTS)
	scan_ret = go_to_point();
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return;
	}
#endif
#if !defined(CONFIG_AUTO_MEASUREMENTS)
	scanner.status = WaitingForContinuation;
	return;
#endif
}

scanner_return_codes_t get_current_point(struct ScanPoint *new_point)
{
	return_codes_t ret;
#if defined(CONFIG_AUTO_MEASUREMENTS)
	int out_val;
	scanner_return_codes_t scan_ret;

	scan_ret = perform_meas(&out_val);
	if (scan_ret != SCAN_SUCCESS) {
		return SCAN_DRIVER_ERROR;
	}
#endif

	uint32_t pos_value;

	ret = position_get(&pos_value, scanner.axes[Yaw].channel);
	if (ret != SUCCESS) {
		return SCAN_DRIVER_ERROR;
	}

	new_point->yaw = pos_value;
	ret = position_get(&pos_value, scanner.axes[Pitch].channel);
	if (ret != SUCCESS) {
		return SCAN_DRIVER_ERROR;
	}

	new_point->pitch = pos_value;
#if defined(CONFIG_AUTO_MEASUREMENTS)
	new_point->meas_value = out_val;
#endif
	return SCAN_SUCCESS;
}

#if !defined(CONFIG_AUTO_MEASUREMENTS)
scanner_return_codes_t move_to_next_point(void)
{
	scanner_return_codes_t scan_ret;

	if (scanner.status != WaitingForContinuation) {
		return SCAN_WRONG_STATUS;
	}

	scanner.status = Scanning;

	scan_ret = go_to_point();
	if (scan_ret != SCAN_SUCCESS) {
		scanner.status = Error;
		return SCAN_DRIVER_ERROR;
	}

	return SCAN_SUCCESS;
}
#endif



#if defined(CONFIG_AUTO_MEASUREMENTS)
static void pause_timer_handler_wrapper(struct k_timer *dummy)
{
	k_work_submit(&point_achieved_work);
}
K_TIMER_DEFINE(pause_timer, pause_timer_handler_wrapper, NULL);
#endif

static void wait_for_point_wrapper(struct k_timer *dummy)
{
	k_work_submit(&wait_for_point_work);
}

K_TIMER_DEFINE(wait_for_point_timer, wait_for_point_wrapper, NULL);

scanner_return_codes_t define_scanner(struct ScannerDefinition new_scanner)
{
	return_codes_t ret;

	if (scanner.status != Ready && scanner.status != Uninitialised) {
		return SCAN_WRONG_STATUS;
	}

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

scanner_return_codes_t start_scanner(void)
{
	scanner_return_codes_t scan_ret;

	if (scanner.status != Ready) {
		return SCAN_WRONG_STATUS;
	}

	// printk("Starting!");
	target[Yaw] = scanner.axes[Yaw].start;
	target[Pitch] = scanner.axes[Pitch].start;
	scan_ret = go_to_point();

	if (scan_ret != SCAN_SUCCESS) {
		return scan_ret;
	}

	scanner.status = Scanning;

	return SCAN_SUCCESS;
}

enum ScannerStatus get_status(void)
{
	return scanner.status;
}

scanner_return_codes_t reset_scanner(void)
{
	if (scanner.status == Finished || scanner.status == Error) {
		scanner.status = Ready;

		return SCAN_SUCCESS;
	}

	return SCAN_WRONG_STATUS;
}

scanner_return_codes_t stop_scanner(void)
{
	if (scanner.status == WaitingForContinuation) {
		scanner.status = Finished;

		return SCAN_SUCCESS;
	}

	if (scanner.status == Scanning) {
		scanner.status = Stopping;

		return SCAN_SUCCESS;
	}

	return SCAN_WRONG_STATUS;
}
