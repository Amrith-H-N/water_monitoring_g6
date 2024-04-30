/* pH Meter and Turbidity Sensor Driver */

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

/* Define the UART devices and configurations */
#define UART0_DEVICE_NODE DT_NODELABEL(uart0)  /* For base station communication */
#define UART0_BAUD_RATE 9600

#define UART1_DEVICE_NODE DT_NODELABEL(uart1)  /* For computer output */
#define UART1_BAUD_RATE 115200

/* Define the GPIO pins for sensor data */
#define PH_METER_PIN DT_GPIO_PIN(DT_NODELABEL(ph_meter), gpios)
#define PH_METER_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(ph_meter), gpios)
#define TURBIDITY_SENSOR_PIN DT_GPIO_PIN(DT_NODELABEL(turbidity_sensor), gpios)
#define TURBIDITY_SENSOR_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(turbidity_sensor), gpios)

/* Define the commands and response codes */
#define CMD_FETCH_PH_METER 0x01
#define CMD_FETCH_TURBIDITY_SENSOR 0x02
#define RESP_OK 0x10
#define RESP_ERROR 0x11

/* Ring buffer for UART0 data */
uint8_t uart0_rx_buffer[256];
struct ring_buf uart0_rx_ring_buf;

/* Function prototypes */
void uart0_isr(const struct device *dev, void *user_data);
int fetch_channel(const struct device *dev, uint8_t channel, uint8_t *value);
int get_channel_value(uint8_t channel);
int water_init(const struct device *dev);

/* UART0 interrupt service routine */
void uart0_isr(const struct device *dev, void *user_data)
{
    uint8_t data;

    while (uart_fifo_read(dev, &data, 1)) {
        ring_buf_put(&uart0_rx_ring_buf, data);
    }
}

/* Fetch the requested channel value */
int fetch_channel(const struct device *dev, uint8_t channel, uint8_t *value)
{
    int ret = 0;

    switch (channel) {
    case CMD_FETCH_PH_METER:
        *value = get_channel_value(PH_METER_PIN);
        break;
    case CMD_FETCH_TURBIDITY_SENSOR:
        *value = get_channel_value(TURBIDITY_SENSOR_PIN);
        break;
    default:
        ret = -1; /* Invalid channel */
        break;
    }

    if (ret == 0) {
        uart_poll_out(UART0_DEVICE_NODE, &RESP_OK, 1);
        uart_poll_out(UART0_DEVICE_NODE, value, 1);
    } else {
        uart_poll_out(UART0_DEVICE_NODE, &RESP_ERROR, 1);
    }

    return ret;
}

/* Get the value from the specified channel */
int get_channel_value(uint8_t channel)
{
    int value = 0;

    /* Read the analog value from the specified channel */
    value = gpio_get_value(channel);

    /* Display the value on UART1 (computer output) */
    char output[20];
    snprintk(output, sizeof(output), "Sensor value: %d\r\n", value);
    uart_poll_out(UART1_DEVICE_NODE, output, strlen(output));

    return value;
}

/* Initialize the sensor driver */
int water_init(const struct device *dev)
{
    uart_irq_callback_set(UART0_DEVICE_NODE, uart0_isr);

    /* Configure UART0 for base station communication */
    const struct device *uart0_dev = device_get_binding(DT_LABEL(UART0_DEVICE_NODE));
    if (!uart0_dev) {
        return -1;
    }

    /* Configure UART1 for computer output */
    const struct device *uart1_dev = device_get_binding(DT_LABEL(UART1_DEVICE_NODE));
    if (!uart1_dev) {
        return -1;
    }

    /* Configure the GPIO pins for sensor data */
    gpio_pin_configure(device_get_binding(DT_GPIO_LABEL(DT_NODELABEL(ph_meter), gpios)),
                       PH_METER_PIN, GPIO_INPUT | PH_METER_FLAGS);
    gpio_pin_configure(device_get_binding(DT_GPIO_LABEL(DT_NODELABEL(turbidity_sensor), gpios)),
                       TURBIDITY_SENSOR_PIN, GPIO_INPUT | TURBIDITY_SENSOR_FLAGS);

    /* Initialize the ring buffer for UART0 data */
    ring_buf_init(&uart0_rx_ring_buf, sizeof(uart0_rx_buffer), uart0_rx_buffer);

    return 0;
}
