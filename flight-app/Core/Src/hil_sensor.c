#include "hil_sensor.h"

#include <float.h>
#include <string.h>

#define SENSOR_DECODED_SIZE 56U
#define SENSOR_CRC_OFFSET   52U
#define SENSOR_PAYLOAD_SIZE 36U
#define SENSOR_VERSION      1U
#define SENSOR_MESSAGE_TYPE 1U

_Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "HIL sensor readings require IEEE 754 binary32");

static uint32_t read_u32_le(const uint8_t *bytes)
{
  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U) |
         ((uint32_t)bytes[2] << 16U) | ((uint32_t)bytes[3] << 24U);
}

static float read_float_le(const uint8_t *bytes)
{
  uint32_t bits = read_u32_le(bytes);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static uint32_t crc32(const uint8_t *bytes, size_t length)
{
  uint32_t crc = UINT32_MAX;
  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= bytes[index];
    for (unsigned int bit = 0U; bit < 8U; ++bit)
    {
      uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
    }
  }
  return crc ^ UINT32_MAX;
}

static bool decode_cobs(const uint8_t *encoded, uint8_t *decoded)
{
  size_t input = 0U;
  size_t output = 0U;
  while (input < HIL_SENSOR_ENCODED_SIZE)
  {
    /* Delimiters never enter encoded[]. A frame this short cannot use 0xFF. */
    size_t count = (size_t)encoded[input++] - 1U;
    if (count > HIL_SENSOR_ENCODED_SIZE - input ||
        count > SENSOR_DECODED_SIZE - output)
    {
      return false;
    }

    memcpy(&decoded[output], &encoded[input], count);
    input += count;
    output += count;

    if (input < HIL_SENSOR_ENCODED_SIZE)
    {
      if (output == SENSOR_DECODED_SIZE)
      {
        return false;
      }
      decoded[output++] = 0U;
    }
  }
  return output == SENSOR_DECODED_SIZE;
}

static bool decode_frame(const uint8_t *encoded, HilSensorSample *sample)
{
  uint8_t decoded[SENSOR_DECODED_SIZE];
  if (!decode_cobs(encoded, decoded) ||
      decoded[0] != SENSOR_VERSION || decoded[1] != SENSOR_MESSAGE_TYPE ||
      decoded[2] != SENSOR_PAYLOAD_SIZE || decoded[3] != 0U ||
      crc32(decoded, SENSOR_CRC_OFFSET) != read_u32_le(&decoded[SENSOR_CRC_OFFSET]))
  {
    return false;
  }

  sample->sample_seq = read_u32_le(&decoded[4]);
  sample->sample_time_us = read_u32_le(&decoded[8]) |
                           ((uint64_t)read_u32_le(&decoded[12]) << 32U);
  for (size_t axis = 0U; axis < 3U; ++axis)
  {
    sample->gyro_rad_s[axis] = read_float_le(&decoded[16U + 4U * axis]);
    sample->accel_m_s2[axis] = read_float_le(&decoded[28U + 4U * axis]);
    sample->mag_uT[axis] = read_float_le(&decoded[40U + 4U * axis]);
  }
  return true;
}

bool HilSensor_PushByte(HilSensorParser *parser, uint8_t byte, HilSensorSample *sample)
{
  if (byte == 0U)
  {
    bool complete = parser->length == HIL_SENSOR_ENCODED_SIZE &&
                    !parser->discard_until_delimiter;
    parser->length = 0U;
    parser->discard_until_delimiter = false;
    return complete && decode_frame(parser->encoded, sample);
  }

  if (parser->length < HIL_SENSOR_ENCODED_SIZE)
  {
    parser->encoded[parser->length++] = byte;
  }
  else
  {
    parser->discard_until_delimiter = true;
  }
  return false;
}
