#include <zephyr.h>
#include <drivers/uart.h>

const struct device *uart_rx_dev;
const struct device *uart_tx_dev;

void main(void) {
    uart_rx_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    uart_tx_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    while (1) {
        // Receive data from the sensor node using UART0
        int sensor_data;
        uart_rx(uart_rx_dev, (uint8_t *)&sensor_data, sizeof(sensor_data), SYS_FOREVER_MS);

        // Process the received sensor data
        process_sensor_data(sensor_data);

        // Transmit a command to the sensor node using UART1
        int command_data = generate_command();
        uart_tx(uart_tx_dev, (uint8_t *)&command_data, sizeof(command_data), SYS_FOREVER_MS);

        k_msleep(1000);
    }
}