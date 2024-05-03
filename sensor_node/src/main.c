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
#include <zephyr/sys/ring_buffer.h>
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

#define MSG_SIZE 256

/* queue to store up to 10 messages (aligned to 1-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Ring buffer for UART0 data */
#define MY_RING_BUF_BYTES 1024

struct my_uart_data {
  uint8_t buf[MY_RING_BUF_BYTES];
  struct ring_buf rb;
} uart_data;
/*
 *
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data) {
  int rx = 0;
  uint8_t *dst;
  uint32_t partial_size = 0;
  uint32_t total_size = 0;

  if (!uart_irq_update(uart_dev)) {
    return;
  }

  if (!uart_irq_rx_ready(uart_dev)) {
    return;
  }
  k_mutex_lock(&my_mutex, K_FOREVER);
  while (uart_irq_update(uart_dev) && uart_irq_rx_ready(uart_dev)) {
    if (!partial_size) {
      partial_size = ring_buf_put_claim(&uart_data.rb, &dst, 1);
    }

    rx = uart_fifo_read(uart_dev, dst, partial_size);
    if (rx <= 0) {
      continue;
    }
    dst += rx;
    total_size += rx;
    partial_size -= rx;
  }
  ring_buf_put_finish(&uart_data.rb, total_size);

  k_mutex_unlock(&my_mutex);
  k_sem_give(&uart_sem);
}

void send_uart() {
  while (1) {
    k_sem_take(&uart_sem, K_FOREVER);
    k_mutex_lock(&my_mutex, K_FOREVER);
    uint8_t data;
    size_t len;

    if (ring_buf_size_get(&uart_data.rb) >= 1) {
      len = ring_buf_get(&uart_data.rb, &data, sizeof(data));
      /* Process the received data (8 bytes) */
      /* ... */
      printk("cmd = %c", data);
      // for (int i = 0; i < 8; i++) printk("cmd = %c", data[i]);
    }

    switch (data) {
      case get_turbidity:
        printk("turb : %d\n", sensors[data - get_turbidity].reading);
        break;
      case get_ph:
        printk("ph : %d\n", sensors[data - get_turbidity].reading);
        break;
      case get_pressure:
        printk("pressure : %d\n", sensors[data - get_turbidity].reading);
        break;
      case get_all:
        printk("%d %d %d\n", sensors[get_turbidity].reading,
               sensors[get_ph].reading, sensors[get_pressure].reading);
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
  ring_buf_init(&uart_data.rb, sizeof(uart_data.buf), uart_data.buf);
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