#ifndef TURBIDITYSENSOR
#define TURBIDITYSENSOR

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

float AnalogRead(int channel);
#define BAD_ANALOG_READ -123

#ifdef __cplusplus
}
#endif

#endif