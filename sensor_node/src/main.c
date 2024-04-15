/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

const struct device *uart_tx_dev;
const struct device *uart_rx_dev;

void main(void) {
    uart_tx_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    uart_rx_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    while (1) {
        // Transmit data to the base station using UART0
        int sensor_data = read_sensor();
        uart_tx(uart_tx_dev, (uint8_t *)&sensor_data, sizeof(sensor_data), SYS_FOREVER_MS);

        // Receive data from the base station using UART1
        int command_data;
        uart_rx(uart_rx_dev, (uint8_t *)&command_data, sizeof(command_data), SYS_FOREVER_MS);

        // Process the received command data
        process_command(command_data);

        k_msleep(1000);
    }
}
