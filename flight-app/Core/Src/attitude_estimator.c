#include "attitude_estimator.h"

#include <math.h>
#include <stddef.h>

#define MIN_HEADING_SINE 1.0e-3f

static bool normalize_vector(float *vector, size_t length)
{
  float largest = 0.0f;
  for (size_t index = 0U; index < length; ++index)
  {
    if (!isfinite(vector[index]))
    {
      return false;
    }
    largest = fmaxf(largest, fabsf(vector[index]));
  }
  if (largest == 0.0f)
  {
    return false;
  }

  /* Scale before squaring to keep the sum in range. */
  float sum = 0.0f;
  for (size_t index = 0U; index < length; ++index)
  {
    vector[index] /= largest;
    sum += vector[index] * vector[index];
  }
  float inverse_norm = 1.0f / sqrtf(sum);
  for (size_t index = 0U; index < length; ++index)
  {
    vector[index] *= inverse_norm;
  }
  return true;
}

static void cross_product(const float left[3], const float right[3], float result[3])
{
  result[0] = left[1] * right[2] - left[2] * right[1];
  result[1] = left[2] * right[0] - left[0] * right[2];
  result[2] = left[0] * right[1] - left[1] * right[0];
}

static void quaternion_from_ned_axes(const float north[3], const float east[3],
                                    const float down[3], float q[4])
{
  /* NED axes expressed in body coordinates form the rows of body-to-NED. */
  float trace = north[0] + east[1] + down[2];
  if (trace > 0.0f)
  {
    float scale = 2.0f * sqrtf(1.0f + trace);
    q[0] = 0.25f * scale;
    q[1] = (down[1] - east[2]) / scale;
    q[2] = (north[2] - down[0]) / scale;
    q[3] = (east[0] - north[1]) / scale;
  }
  else if (north[0] >= east[1] && north[0] >= down[2])
  {
    float scale = 2.0f * sqrtf(1.0f + north[0] - east[1] - down[2]);
    q[0] = (down[1] - east[2]) / scale;
    q[1] = 0.25f * scale;
    q[2] = (north[1] + east[0]) / scale;
    q[3] = (north[2] + down[0]) / scale;
  }
  else if (east[1] >= down[2])
  {
    float scale = 2.0f * sqrtf(1.0f - north[0] + east[1] - down[2]);
    q[0] = (north[2] - down[0]) / scale;
    q[1] = (north[1] + east[0]) / scale;
    q[2] = 0.25f * scale;
    q[3] = (east[2] + down[1]) / scale;
  }
  else
  {
    float scale = 2.0f * sqrtf(1.0f - north[0] - east[1] + down[2]);
    q[0] = (east[0] - north[1]) / scale;
    q[1] = (north[2] + down[0]) / scale;
    q[2] = (east[2] + down[1]) / scale;
    q[3] = 0.25f * scale;
  }
}

bool AttitudeEstimator_QuaternionFromAccelMag(const float accel_m_s2[3],
                                              const float mag_uT[3],
                                              float q_body_to_ned[4])
{
  float down[3] = {-accel_m_s2[0], -accel_m_s2[1], -accel_m_s2[2]};
  float magnetic[3] = {mag_uT[0], mag_uT[1], mag_uT[2]};
  if (!normalize_vector(down, 3U) || !normalize_vector(magnetic, 3U))
  {
    return false;
  }

  float east[3];
  cross_product(down, magnetic, east);
  float heading_sine_squared = east[0] * east[0] + east[1] * east[1] + east[2] * east[2];
  /* The cross product of the unit vectors has magnitude sin(angle). */
  if (heading_sine_squared < MIN_HEADING_SINE * MIN_HEADING_SINE)
  {
    return false;
  }
  float inverse_east_norm = 1.0f / sqrtf(heading_sine_squared);
  for (size_t axis = 0U; axis < 3U; ++axis)
  {
    east[axis] *= inverse_east_norm;
  }

  float north[3];
  cross_product(east, down, north);
  float attitude[4];
  quaternion_from_ned_axes(north, east, down, attitude);
  if (!normalize_vector(attitude, 4U))
  {
    return false;
  }
  for (size_t index = 0U; index < 4U; ++index)
  {
    q_body_to_ned[index] = attitude[index];
  }
  return true;
}

static void multiply_quaternions(const float left[4], const float right[4], float result[4])
{
  float w = left[0], x = left[1], y = left[2], z = left[3];
  float a = right[0], b = right[1], c = right[2], d = right[3];

  result[0] = w * a - x * b - y * c - z * d;
  result[1] = w * b + x * a + y * d - z * c;
  result[2] = w * c - x * d + y * a + z * b;
  result[3] = w * d + x * c - y * b + z * a;
}

bool AttitudeEstimator_Init(AttitudeEstimator *state, const float initial_q[4],
                            const float gyro_bias_rad_s[3], uint64_t max_gap_us)
{
  AttitudeEstimator next = {
    .q_body_to_ned = {initial_q[0], initial_q[1], initial_q[2], initial_q[3]},
    .max_gap_us = max_gap_us,
    .initialized = true,
  };
  if (max_gap_us == 0U || !normalize_vector(next.q_body_to_ned, 4U))
  {
    return false;
  }
  for (size_t axis = 0U; axis < 3U; ++axis)
  {
    if (!isfinite(gyro_bias_rad_s[axis]))
    {
      return false;
    }
    next.gyro_bias_rad_s[axis] = gyro_bias_rad_s[axis];
  }

  *state = next;
  return true;
}

AttitudeEstimatorResult AttitudeEstimator_Update(AttitudeEstimator *state,
                                                 uint64_t sample_time_us,
                                                 const float gyro_rad_s[3])
{
  if (!state->initialized)
  {
    return ATTITUDE_ESTIMATOR_NEEDS_INIT;
  }

  float rate_rad_s[3];
  for (size_t axis = 0U; axis < 3U; ++axis)
  {
    rate_rad_s[axis] = gyro_rad_s[axis] - state->gyro_bias_rad_s[axis];
    if (!isfinite(rate_rad_s[axis]))
    {
      return ATTITUDE_ESTIMATOR_INVALID_INPUT;
    }
  }

  AttitudeEstimatorResult result = ATTITUDE_ESTIMATOR_PRIMED;
  if (state->primed)
  {
    if (sample_time_us <= state->previous_time_us)
    {
      return ATTITUDE_ESTIMATOR_NONINCREASING_TIME;
    }
    uint64_t elapsed_us = sample_time_us - state->previous_time_us;
    if (elapsed_us > state->max_gap_us)
    {
      state->initialized = false;
      return ATTITUDE_ESTIMATOR_DISCONTINUITY;
    }

    float half_dt_s = (float)elapsed_us * 0.5e-6f;
    float x = state->previous_rate_rad_s[0] * half_dt_s;
    float y = state->previous_rate_rad_s[1] * half_dt_s;
    float z = state->previous_rate_rad_s[2] * half_dt_s;
    float half_angle = hypotf(hypotf(x, y), z);
    if (!isfinite(half_angle))
    {
      return ATTITUDE_ESTIMATOR_INVALID_INPUT;
    }

    /* sin(angle) / angle tends to one as the angle tends to zero. */
    float scale = half_angle == 0.0f ? 1.0f : sinf(half_angle) / half_angle;
    float delta[4] = {cosf(half_angle), x * scale, y * scale, z * scale};
    float attitude[4];
    multiply_quaternions(state->q_body_to_ned, delta, attitude);
    if (!normalize_vector(attitude, 4U))
    {
      return ATTITUDE_ESTIMATOR_INVALID_INPUT;
    }
    for (size_t index = 0U; index < 4U; ++index)
    {
      state->q_body_to_ned[index] = attitude[index];
    }
    result = ATTITUDE_ESTIMATOR_UPDATED;
  }

  for (size_t axis = 0U; axis < 3U; ++axis)
  {
    state->previous_rate_rad_s[axis] = rate_rad_s[axis];
  }
  state->previous_time_us = sample_time_us;
  state->primed = true;
  return result;
}
