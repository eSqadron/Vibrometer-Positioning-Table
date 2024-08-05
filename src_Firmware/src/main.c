// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024 Maciej Baczmanski, Michal Kawiak, Jakub Mazur

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
	init_pwm_motor_driver();

#if defined(CONFIG_BT_SUPPORT)
	init_bt();
#endif

#if defined(CONFIG_AUTO_MEASUREMENTS)
	init_adc_scan();
#endif
}
