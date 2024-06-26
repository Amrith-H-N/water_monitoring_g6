#ifndef ANALOG_SENSORS_H_
#define ANALOG_SENSORS_H_

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

void rd_adc(uint32_t *data);
void adcsensor_init(void);

#endif
