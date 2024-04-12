#include <zephyr/drivers/adc.h>
#include "scan_adc.h"


static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static bool is_initialised = false;

int16_t buf;
struct adc_sequence sequence = {
	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),
	//Optional
	//.calibrate = true,
};

void init_adc_scan(void)
{
	int err;
	// TODO - asserts
        if (!adc_is_ready_dt(&adc_channel)) {
		printk("ADC controller devivce %s not ready", adc_channel.dev->name);
		return;
	}

	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		printk("Could not setup channel #%d (%d)", 0, err);
		return;
	}

	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		printk("Could not initalize sequnce");
		return;
	}

	is_initialised = true;
}

scanner_return_codes_t perform_meas(int *out_val)
{
	int err;

	if (!is_initialised) {
		return ADC_NOT_INITIALISED;
	}

	err = adc_read(adc_channel.dev, &sequence);
	if (err < 0) {
		return ADC_READ_FAIL;
	}

	*out_val = buf;

	err = adc_raw_to_millivolts_dt(&adc_channel, out_val);

	/* conversion to mV may not be supported, skip if not */
	if (err < 0) {
		return ADC_CONVERSION_FAIL;
	}

	return SCAN_SUCCESS;
}
