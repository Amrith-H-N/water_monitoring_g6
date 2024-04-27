#include "ds18b20.h"

/*
Step 1. Initialization
Step 2. ROM Command (followed by any required data
exchange)
Step 3. DS18B20 Function Command (followed by any
required data exchange)
*/

// returns ds18b20 device config from overlay
static const struct device *get_ds18b20_config(void) {
  const struct device *const dev = DEVICE_DT_GET_ANY(maxim_ds18b20);

  if (dev == NULL) {
    /* No such node, or the node does not have status "okay". */
    printk("\n get_ds18b20_dev : no device found.\n");
    return NULL;
  }

  if (!device_is_ready(dev)) {
    printk("\nError: Device \"%s\" is not ready; ", dev->name);
    return NULL;
  }

  return dev;
}

// dump scrathpad to device memory
void ds18b20_write_scratchpad(ds18b20_config_t *ds18b20, ds18b20_data_t *data) {
  uint8_t ram_data[4] = {
      DS18B20_FN_WRITE_SCRATCHPAD_CMD, data->scratchpad.TH_REGISTER,
      data->scratchpad.TL_REGISTER, data->scratchpad.config_register};

  return w1_write_read(ds18b20->addr, &data->config, ram_data, sizeof(ram_data),
                       NULL, 0);
}

// write to scrathpad struct
void ds18b20_set_resolution(ds18b20_config_t *ds18b20, ds18b20_data_t *data) {
  // config register bit 7-0 : res r1 r0 res ... res

  data->scratchpad.config_register |= ds18b20->resolution;
}

// configure the device
void ds18b20_configure() {
  const struct device *dev = get_ds18b20_config();

  if (dev == NULL) {
    return 0;
  }

  ds18b20_config_t *ds18b20 = dev->config;  // device config copy to struct
  ds18b20_data_t *data = dev->data;

  // check if any device is present by sending reset pulse
  if (w1_reset_bus(ds18b20->addr) <= 0) {
    printk("No 1-Wire devices found");
    return -1;
  }

  /* read eeprom from device - check datasheet for more info */
  if (w1_rom_to_uint64(&data->config.rom) == 0) {
    (void)w1_read_rom(ds18b20->addr, &data->config.rom);
  }
  ds18b20->resolution = DS18B20_RESOLUTION;
  /* write default configuration */
  ds18b20_set_resolution(ds18b20, data);

  printk("Init DS18B20: ROM=%016llx\n", w1_rom_to_uint64(&data->config.rom));

  return 0;
}
