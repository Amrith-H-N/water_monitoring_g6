/* Application Code */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "sensor_driver.h" /* Include the sensor driver header file */

#define SLEEP_TIME_MS 1000
#define ACK_BYTE 0x20 /* Acknowledgment byte */

const struct device *uart0_dev; /* UART device for communication */

void uart0_isr(const struct device *dev, void *user_data)
{
    uint8_t data;

    while (uart_fifo_read(dev, &data, 1)) {
        /* Received data from the sensor node */
        if (data == RESP_OK) {
            /* Receive the sensor value */
            uint8_t value;
            uart_fifo_read(dev, &value, 1);

            printk("Received sensor value: %d\n", value);

            /* Send acknowledgment */
            uart_poll_out(uart0_dev, &ACK_BYTE, 1);
        } else if (data == RESP_ERROR) {
            printk("Error occurred while fetching sensor value\n");
        }
    }
}

void main(void)
{
    const struct device *sensor_dev;
    int ret;

    /* Initialize the sensor driver */
    sensor_dev = device_get_binding("SENSOR_DRIVER");
    if (sensor_dev == NULL) {
        printk("Failed to get sensor device\n");
        return;
    }

    /* Configure and initialize the sensor */
    ret = sensor_init(sensor_dev);
    if (ret < 0) {
        printk("Failed to initialize sensor\n");
        return;
    }

    /* Get the UART device for communication */
    uart0_dev = device_get_binding(DT_LABEL(UART0_DEVICE_NODE));
    if (uart0_dev == NULL) {
        printk("Failed to get UART device\n");
        return;
    }

    /* Set UART interrupt callback */
    uart_irq_callback_set(uart0_dev, uart0_isr);

    while (1) {
        /* Fetch pH meter value */
        uart_poll_out(uart0_dev, &CMD_FETCH_PH_METER, 1);

        /* Fetch turbidity sensor value */
        uart_poll_out(uart0_dev, &CMD_FETCH_TURBIDITY_SENSOR, 1);

        k_msleep(SLEEP_TIME_MS);
    }
}
