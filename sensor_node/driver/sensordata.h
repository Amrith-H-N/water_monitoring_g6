#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#define num_sensors 3

enum cmd { get_turbidity = '0', get_ph, get_pressure, get_all };

typedef struct {
  uint8_t id;
  uint32_t reading;
} sensor_t;

extern sensor_t sensors[num_sensors];
#endif