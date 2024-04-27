#ifndef DS18B20_H_
#define DS18B20_H_
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

#include "sensordata.h"

#define DS18B20_RESOLUTION 10  // 10 bit resolution

// DS18B20 Function Commands
#define DS18B20_FN_CONVERT_T_CMD 0x44
#define DS18B20_FN_WRITE_SCRATCHPAD_CMD 0x4E
#define DS18B20_FN_READ_SCRATCHPAD_CMD 0xBE
#define DS18B20_FN_COPY_SCRATCHPAD_CMD 0x48
#define DS18B20_FN_RECALL_EEPROM_CMD 0xB8
#define DS18B20_FN_READ_POWER_SUPPLY_CMD 0xB4

// ROM Commands
#define DS18B20_SEARCH_ROM_CMD 0xF0
#define DS18B20_READ_ROM_CMD 0x33
#define DS18B20_MATCH_ROM_CMD 0x55
#define DS18B20_SKIP_ROM_CMD 0xCC
#define DS18B20_ALARMSEARCH_ROM_CMD 0xEC

typedef struct {
  const struct device *addr;  // device tree address
  uint8_t family;
  uint8_t resolution;
} ds18b20_config_t;
// RAM sscratchpad structure
typedef struct ds18b20_scratchpad {
  int16_t temperature;  // in degree celcius
  uint8_t TH_REGISTER;
  uint8_t TL_REGISTER;
  uint8_t config_register;
  uint8_t reserved_padding[3];
  uint8_t crc;
} ds18b20_scratchpad_t;
//
typedef struct ds18b20_data {
  struct w1_slave_config config;    // rom data
  ds18b20_scratchpad_t scratchpad;  // our scratchpad
} ds18b20_data_t;

// void ds18b20_set_resolution(dev, ds18b20->resolution);

#endif