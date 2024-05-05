#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#include "analogsensors.h"
#include "bme680.h"

#define LOG_MODULE_NAME SENSOR_NODE_APP

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* size of stack area used by each thread */
#define STACKSIZE 512

/* scheduling priority used by each thread */
#define PRIORITY 7

// semaphore for button task
K_SEM_DEFINE(uart_sem, 0, 1);
// mutex to protect curr_led
K_MUTEX_DEFINE(my_mutex);

#define uart_delay 200
/* change this to any other UART peripheral if desired */

#define MSG_SIZE 256

/* queue to store up to 10 messages (aligned to 1-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

// static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct uart_config uart_cfg = {.baudrate = 115200,
                                     .parity = UART_CFG_PARITY_NONE,
                                     .stop_bits = UART_CFG_STOP_BITS_1,
                                     .data_bits = UART_CFG_DATA_BITS_8,
                                     .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

/* Ring buffer for UART0 data */
#define MY_RING_BUF_BYTES 1024

struct my_uart_data {
  uint8_t buf[MY_RING_BUF_BYTES];
  struct ring_buf rb;
} uart_data;

#define num_sensors 3

enum cmd { get_turbidity = 0, get_ph, get_temp, get_all };

typedef struct {
  uint16_t val1;
  uint16_t val2;
} sensor_t;

sensor_t sensors[num_sensors] = {{0, 0}, {0, 0}, {0, 0}};
const float ph_calibration = 0;
const float turb_calibration = 0;

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

  if (!uart_irq_update(uart)) {
    return;
  }

  if (!uart_irq_rx_ready(uart)) {
    return;
  }
  k_mutex_lock(&my_mutex, K_FOREVER);
  while (uart_irq_update(uart) && uart_irq_rx_ready(uart)) {
    if (!partial_size) {
      partial_size = ring_buf_put_claim(&uart_data.rb, &dst, 1);
    }

    rx = uart_fifo_read(uart, dst, partial_size);
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
    int err;

    uint8_t data;
    size_t len;

    if (ring_buf_size_get(&uart_data.rb) >= 1) {
      len = ring_buf_get(&uart_data.rb, &data, sizeof(data));
      /* Process the received data (8 bytes) */
      /* ... */
      // printk("cmd = %x", data);
      //   for (int i = 0; i < 8; i++) printk("cmd = %c", data[i]);
    }

    int index = 0;
    int length = 0;

    switch (data) {
      case get_turbidity:
        index = 0;
        length = 4;

        break;
      case get_ph:
        index = 1;
        length = 4;
        break;
      case get_temp:
        index = 2;
        length = 4;
        break;
      case get_all:
        index = 0;
        length = 12;  // 4 (32bits)* 3 (sensors)
        break;
      default:
        break;
    }
    uint8_t *buffer = &sensors[index];
    for (int i = 0; i < length; i++) uart_poll_out(uart, buffer[i]);
    k_mutex_unlock(&my_mutex);
    k_msleep(1000);
  }
}

int uart_init() {
  if (!device_is_ready(uart)) {
    printk("UART device not found!");
    return 0;
  }

  int err = uart_configure(uart, &uart_cfg);

  if (err == -ENOSYS) {
    return -ENOSYS;
  }

  /* configure interrupt and callback to receive data */
  int ret = uart_irq_callback_user_data_set(uart, serial_cb, NULL);

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
  uart_irq_rx_enable(uart);
}
void update_sensordata() {
  uint32_t data[2];
  while (1) {
    // get analog sensor data
    rd_adc(data);
    // get temperature
    data[2] = get_temperature();
    k_mutex_lock(&my_mutex, K_FOREVER);
    // do conversion and store

    // ph adc channel 1
    float adc1_temp =
        ((((float)data[1] / 4096.0) * 3.3)) * 3.5 + ph_calibration;
    sensors[1].val1 = (int16_t)adc1_temp;
    sensors[1].val2 = (int16_t)((float)(adc1_temp - (int16_t)adc1_temp) *
                                1000);  // 2 decimal points

    // turbidity y=-1120x^2+5742.3x-4352.9 , where x is voltage , y is in NTU
    float x = ((((float)data[0] / 4096.0) * 4.5)) + turb_calibration;
    float y = (-1120 * (x * x)) + (5742.3 * (x)) - 4352.9;
    sensors[0].val1 = (int16_t)y;
    sensors[0].val2 =
        (int16_t)((float)(y - (int16_t)y) * 1000);  // 2 decimal points

    // temperature from bme680
    float bme_temp = (float)data[2] / 100.0;
    sensors[2].val1 = (uint16_t)bme_temp;
    sensors[2].val2 = (uint16_t)((float)(bme_temp - (uint16_t)bme_temp) *
                                 1000);  // 2 decimal points

    // printk("\nph = %f", adc1_temp);
    // printk("\nturbidity = %f", y);
    // printk("\ntemperature = %d", data[2]);
    k_mutex_unlock(&my_mutex);
    k_msleep(1000);
  }
}

int main(void) {
  config_sensor();
  adcsensor_init();
  uart_init();

  return 0;
}

K_THREAD_DEFINE(send_uart_task, STACKSIZE, send_uart, NULL, NULL, NULL, 0, 0,
                1000);
// update sensor data
K_THREAD_DEFINE(update_sensordata_task, 2048, update_sensordata, NULL, NULL,
                NULL, 0, 0, 1000);