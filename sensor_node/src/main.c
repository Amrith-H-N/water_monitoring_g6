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

#include "analogsensors.h"
#include "ds18b20.h"

#define LOG_MODULE_NAME SENSOR_NODE_APP

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

// semaphore for button task
K_SEM_DEFINE(uart_sem, 0, 1);
// mutex to protect curr_led
K_MUTEX_DEFINE(my_mutex);

#define uart_delay 200
/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 8

/* queue to store up to 10 messages (aligned to 1-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;

/*
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data) {
  uint8_t c;

  if (!uart_irq_update(uart_dev)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }
  k_mutex_lock(&my_mutex, K_FOREVER);
  /* read until FIFO empty */
  if (uart_fifo_read(uart_dev, &c, 1) == 1) {
    if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
      rx_buf[rx_buf_pos++] = c;
    } else
      rx_buf_pos = 0;
    /* else: characters beyond buffer size are dropped */
  }
  k_mutex_unlock(&my_mutex);
  k_sem_give(&uart_sem);
}

void send_uart() {
  while (1) {
    k_sem_take(&uart_sem, K_FOREVER);
    k_mutex_lock(&my_mutex, K_FOREVER);
    char value = rx_buf[rx_buf_pos - 1];
    // printk("cmd = %c", value);
    switch (value) {
      case get_turbidity:
        printk("%d\n", sensors[value - get_turbidity].reading);
        break;
      case get_ph:
        printk("%d\n", sensors[value - get_turbidity].reading);
        break;
      case get_pressure:
        printk("%d\n", sensors[value - get_turbidity].reading);
        break;
      case get_all:
        printk("%d %d %d\n", 20, 21, 22);
        break;
      default:
        break;
    }

    k_mutex_unlock(&my_mutex);
    k_msleep(uart_delay);
  }
}

int uart_init() {
  if (!device_is_ready(uart_dev)) {
    printk("UART device not found!");
    return 0;
  }

  /* configure interrupt and callback to receive data */
  int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  if (ret < 0) {
    if (ret == -ENOTSUP) {
      printk("Interrupt-driven UART API support not enabled\n");
    } else if (ret == -ENOSYS) {
      printk("UART device does not support interrupt-driven API\n");
    } else {
      printk("Error setting UART callback: %d\n", ret);
    }
    return 0;
  }
  uart_irq_rx_enable(uart_dev);
}

int main(void) {
  analog_sensors_init();
  uart_init();

  return 0;
}

K_THREAD_DEFINE(send_uart_task, STACKSIZE, send_uart, NULL, NULL, NULL, 0, 0,
                0);
// reads turbidity and ph values at the same time
K_THREAD_DEFINE(read_adc_task, STACKSIZE, read_adc, NULL, NULL, NULL, 0, 0, 0);