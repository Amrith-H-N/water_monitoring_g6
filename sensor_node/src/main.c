#include <zephyr.h>
#include <drivers/uart.h>

const struct device *uart_tx_dev;
const struct device *uart_rx_dev;

void process_command(int command_data) {
    // Implement 
    printk("Received command: %d\n", command_data);
}

void main(void) {
    uart_tx_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    uart_rx_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    while (1) {
        // Read sensor data
        int sensor_data = read_sensor();

        // Transmit data to the base station using UART0
        uart_tx(uart_tx_dev, (uint8_t *)&sensor_data, sizeof(sensor_data), SYS_FOREVER_MS);
        printk("Sent sensor data: %d\n", sensor_data);

        // Receive data from the base station using UART1
        int command_data;
        int ret = uart_rx(uart_rx_dev, (uint8_t *)&command_data, sizeof(command_data), SYS_FOREVER_MS);
        if (ret == 0) {
            process_command(command_data);
            // Send acknowledgement
            uint8_t ack = 0x06; // ASCII ACK character
            uart_tx(uart_tx_dev, &ack, 1, SYS_FOREVER_MS);
            printk("Sent acknowledgement\n");
        }

        k_msleep(1000);
    }
}
