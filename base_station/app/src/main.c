/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <app/drivers/sensor/sensor.h>

static const struct device *fpreader = DEVICE_DT_GET(DT_NODELABEL(fpreader));
static const struct device *lock = DEVICE_DT_GET(DT_NODELABEL(lock));

/*******************************************************************************
 * Entry point
 ******************************************************************************/

int main(void) { return 0; }
