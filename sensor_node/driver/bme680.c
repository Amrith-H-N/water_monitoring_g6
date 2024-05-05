#include "bme680.h"

/**
 * @defgroup drivers Drivers
 * @{
 *
 * @brief This is a group of all driver-related code.
 *
 * This group contains all the driver classes, functions, and variables.
 * It's a central place to find everything related to drivers in this project.
 */

/**
 * @defgroup drivers_bme680 drivers
 * @ingroup drivers
 * @{
 *
 * @brief A custom driver for Water monitoring sensor
 *
 *
 *
 */
/* Temperature calibration parameters */
#define BME680_T1_LSB 0xE9
#define BME680_T1_MSB 0xEA
#define BME680_T2_LSB 0x8A
#define BME680_T2_MSB 0x8B
#define BME680_T3_LSB 0x8C

#define BME680_I2C_ADDR 0x77
#define BME680_I2C_DEV I2C_DT_SPEC_GET(DT_NODELABEL(i2c0))

// get device address from dt
const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
uint8_t par_t1[2] = {0, 0}, par_t2[2] = {0, 0}, par_t3[2] = {0, 0};
volatile uint8_t val = 0;
uint16_t _t1 = 0;
uint16_t _t2 = 0;
uint16_t _t3 = 0;
/**
 * @brief gets the temperature from bme680
 *
 *
 */
int16_t get_temperature() {
  // oversampling temperature setting
  volatile uint8_t temp = val | 1 << 0;
  i2c_reg_write_byte(i2c_dev, BME680_I2C_ADDR, BME680_CTRL_MEAS, temp);

  // read it back for debugging
  temp = 0;
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_CTRL_MEAS, &temp);
  // printk("tBME680_CTRL_MEAS: %d \n", temp);
  k_sleep(K_MSEC(10));

  /* Read the temperature registers */
  uint8_t temp_msb = 0, temp_lsb = 0, temp_xlsb = 0;
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_TEMP_MSB, &temp_msb);
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_TEMP_LSB, &temp_lsb);
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_TEMP_XLSB, &temp_xlsb);
  // printk("temp_msb: %x \n", temp_msb);
  // printk("temp_lsb: %x \n", temp_lsb);
  // printk("temp_xlsb: %x \n", temp_xlsb);
  /* Convert the raw temperature data to actual temperature */
  volatile uint32_t temp_xlsb_val = (uint32_t)(temp_xlsb & (0xf0));

  // combine all values to a single variable
  volatile uint32_t temp_raw = (((uint32_t)temp_msb) << 12) +
                               (((uint32_t)temp_lsb) << 4) +
                               (uint32_t)temp_xlsb_val;

  // printk("temp_xlsb raw: %x \n", temp_xlsb_val);
  // printk("temp_raw: %x \n", temp_raw);

  // temperature computation based on calibration
  volatile int64_t temp_var1 = ((int32_t)temp_raw >> 3) - ((int32_t)_t1 << 1);
  volatile int64_t temp_var2 = (temp_var1 * (int32_t)_t2) >> 11;
  volatile int64_t temp_var3 =
      (((temp_var1 >> 1) * (temp_var1 >> 1) >> 12)) * ((int32_t)_t3 << 4) >> 14;
  volatile int64_t temp_fine = temp_var2 + temp_var3;
  // printk("###Temp_fine: %lld \n", temp_fine);
  volatile int16_t temp_comp = ((temp_fine * 5) + 128) >> 8;
  // printk("Temperature: %d °C\n", temp_comp);

  return temp_comp;
  /* Print the temperature */
  // printk("Temperature: %6.6f °C\n", (float)temp_comp / 100);
}

/**
 * @brief configures bme680
 *
 *
 */
void config_sensor() {
  if (!device_is_ready(i2c_dev)) {
    printk("I2C device not ready\n");
    return;
  }

  /* Initialize the sensor */
  uint8_t chip_id;
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_ID, &chip_id);
  printk("id: %d \n", chip_id);

  /* Soft Reset the sensor */
  i2c_reg_write_byte(i2c_dev, BME680_I2C_ADDR, BME680_RESET, 0xB6);
  k_sleep(K_MSEC(200));

  /* Configure the sensor */
  i2c_reg_write_byte(i2c_dev, BME680_I2C_ADDR, BME680_CONFIG,
                     0x04 << 2); /* Normal mode, 5ms IIR filter */

  i2c_burst_read(i2c_dev, BME680_I2C_ADDR, BME680_T1_LSB, par_t1, 2);
  i2c_burst_read(i2c_dev, BME680_I2C_ADDR, BME680_T2_LSB, par_t2, 2);
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_T3_LSB, par_t3);

  _t1 = *((uint16_t *)par_t1);
  _t2 = *((uint16_t *)par_t2);
  _t3 = par_t3[0];

  val = 4 << 5;
  i2c_reg_write_byte(i2c_dev, BME680_I2C_ADDR, BME680_CTRL_MEAS, val);
  volatile uint8_t temp = 0;
  i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, BME680_CTRL_MEAS, &temp);
}

/** @} */
/** @} */