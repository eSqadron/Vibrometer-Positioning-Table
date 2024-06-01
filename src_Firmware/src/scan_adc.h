/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Jakub Mazur
 *
 */
#if defined(CONFIG_AUTO_MEASUREMENTS)
# include "scanner_return_codes.h"
void init_adc_scan(void);
scanner_return_codes_t perform_meas(int *out_val);
#endif
