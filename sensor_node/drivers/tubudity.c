#include <zephyr.h>
#include <drivers/adc.h>
#include <stdio.h>

#define TURBIDITY_SENSOR_ADC_CHANNEL 0
#define SAMPLE_DELAY_MS 500

const struct adc_channel_cfg my_channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = TURBIDITY_SENSOR_ADC_CHANNEL,
};

void main(void)
{
    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
    int ret;

    ret = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (ret) {
        printk("adc_channel_setup failed with code %d\n", ret);
        return;
    }

    while (1) {
        int16_t raw_value;
        float voltage;

        ret = adc_read(adc_dev, &raw_value);
        if (ret) {
            printk("adc_read failed with code %d\n", ret);
            return;
        }

        voltage = (float)raw_value * 3.3f / 4096.0f;
        printf("Turbidity sensor value: %.2f V\n", voltage);

        k_msleep(SAMPLE_DELAY_MS);
    }
}
