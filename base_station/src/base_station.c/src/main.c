#include <zephyr.h>
#include <drivers/uart.h>

const struct device *uart_rx_dev;
const struct device *uart_tx_dev;

void process_sensor_data(int sensor_data) {
    // Implement 
   
    printk("Received sensor data: %d\n", sensor_data);
}

int generate_command() {
    // Implement
    return 1;
}

void main(void) {
    uart_rx_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    uart_tx_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    while (1) {
        // Receive data from the sensor node using UART0
        int sensor_data;
        int ret = uart_rx(uart_rx_dev, (uint8_t *)&sensor_data, sizeof(sensor_data), SYS_FOREVER_MS);
        if (ret == 0) {
            process_sensor_data(sensor_data);
            // Send acknowledgement
            uint8_t ack = 0x06; // ASCII ACK character
            uart_tx(uart_tx_dev, &ack, 1, SYS_FOREVER_MS);
            printk("Sent acknowledgement\n");
        }

        // Transmit a command to the sensor node using UART1
        int command_data = generate_command();
        uart_tx(uart_tx_dev, (uint8_t *)&command_data, sizeof(command_data), SYS_FOREVER_MS);
        printk("Sent command: %d\n", command_data);

        // Wait for acknowledgement from the sensor node
        uint8_t ack;
        ret = uart_rx(uart_rx_dev, &ack, 1, K_SECONDS(1));
        if (ret == 0 && ack == 0x06) {
            printk("Received acknowledgement\n");
        } else {
            printk("Failed to receive acknowledgement\n");
        }

        k_msleep(1000);
    }
}
