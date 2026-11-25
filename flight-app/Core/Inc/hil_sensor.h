#ifndef HIL_SENSOR_H
#define HIL_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HIL_SENSOR_ENCODED_SIZE 57U

typedef struct
{
  uint64_t sample_time_us;
  uint32_t sample_seq;
  float gyro_rad_s[3];
  float accel_m_s2[3];
  float mag_uT[3];
} HilSensorSample;

typedef struct
{
  uint8_t encoded[HIL_SENSOR_ENCODED_SIZE];
  size_t length;
  bool discard_until_delimiter;
} HilSensorParser;

/* Zero-initialize the parser. Feed bytes in order across any transport boundary.
 * Returns true and writes sample only when a complete valid frame ends.
 */
bool HilSensor_PushByte(HilSensorParser *parser, uint8_t byte, HilSensorSample *sample);

#endif
