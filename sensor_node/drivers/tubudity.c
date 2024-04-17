#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define TURBIDITY_SENSOR_PIN 26 // Analog input pin for the turbidity sensor

int main() {
    stdio_init_all();
    adc_init();
    adc_set_voltage_range(3.3f);
    adc_select_input(TURBIDITY_SENSOR_PIN);

    while (true) {
        int raw_value = adc_read();
        float voltage = raw_value * (3.3f / 4095.0f);
        printf("Turbidity sensor value: %f V\n", voltage);
        sleep_ms(500);
    }
}
