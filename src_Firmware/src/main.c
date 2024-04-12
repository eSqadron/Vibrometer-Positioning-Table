// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2023 Maciej Baczmanski, Michal Kawiak, Jakub Mazur

#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>

#if defined(CONFIG_BT_SUPPORT)
#include "ble_gatt_service.h"
#endif

#include <string.h>

#include "driver.h"
#include "storage.h"
#include "scan_adc.h"

#if defined(CONFIG_UART_SHELL_SUPPORT)
#include <zephyr/drivers/uart.h>
#endif

int main(void)
{
	struct Template initial_template;
	return_codes_t error = SUCCESS;

	init_pwm_motor_driver();
	init_storage();
	error = get_current_template(&initial_template);

	if (error == SUCCESS) {// TODO - test it, currently broken by init controller.
		target_speed_set(initial_template.speed, CH0);
	}

	#if defined(CONFIG_BT_SUPPORT)
	init_bt();
	#endif

	init_adc_scan();
}
