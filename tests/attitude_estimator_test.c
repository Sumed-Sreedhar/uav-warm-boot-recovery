#include "attitude_estimator.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static const float stationary[3] = {0.0f, 0.0f, 0.0f};
static const float positive_yaw_rate[3] = {0.0f, 0.0f, 1.57079632679f};
static const float sqrt_half = 0.70710678118f;
static const float gravity = 9.80665f;

static void expect_attitude(const AttitudeEstimator *state, const float expected[4])
{
  for (size_t index = 0U; index < 4U; ++index)
  {
    assert(fabsf(state->q_body_to_ned[index] - expected[index]) < 2.0e-5f);
  }
}

static void test_stationary_and_initial_normalization(void)
{
  AttitudeEstimator state = {0};
  const float scaled_identity[4] = {FLT_MAX, 0.0f, 0.0f, 0.0f};
  assert(AttitudeEstimator_Init(&state, scaled_identity, stationary, 5000U));
  assert(AttitudeEstimator_Update(&state, 0U, stationary) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 5000U, stationary) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, identity);
}

static void test_yaw_and_body_side_multiplication(void)
{
  AttitudeEstimator state = {0};
  const float yaw_90[4] = {sqrt_half, 0.0f, 0.0f, sqrt_half};
  const float roll_90[4] = {sqrt_half, sqrt_half, 0.0f, 0.0f};
  const float roll_then_body_yaw[4] = {0.5f, 0.5f, -0.5f, 0.5f};

  assert(AttitudeEstimator_Init(&state, identity, stationary, 1000000U));
  assert(AttitudeEstimator_Update(&state, 0U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 1000000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_90);

  assert(AttitudeEstimator_Init(&state, roll_90, stationary, 1000000U));
  assert(AttitudeEstimator_Update(&state, 0U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 1000000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, roll_then_body_yaw);
}

static void test_timestamps_and_previous_sample_hold(void)
{
  AttitudeEstimator state = {0};
  const uint64_t start = UINT64_C(1) << 55U;
  const float yaw_45[4] = {0.92387953251f, 0.0f, 0.0f, 0.38268343236f};
  assert(AttitudeEstimator_Init(&state, identity, stationary, 500000U));
  assert(AttitudeEstimator_Update(&state, start, positive_yaw_rate) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, start, stationary) == ATTITUDE_ESTIMATOR_NONINCREASING_TIME);
  assert(AttitudeEstimator_Update(&state, start - 1U, stationary) == ATTITUDE_ESTIMATOR_NONINCREASING_TIME);
  expect_attitude(&state, identity);

  assert(AttitudeEstimator_Update(&state, start + 125000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  assert(AttitudeEstimator_Update(&state, start + 500000U, stationary) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_45);
  assert(AttitudeEstimator_Update(&state, start + 600000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_45);
}

static void test_bias_subtraction(void)
{
  AttitudeEstimator state = {0};
  const float bias[3] = {0.01f, -0.02f, 0.03f};
  const float biased_yaw[3] = {bias[0], bias[1], 1.57079632679f + bias[2]};
  const float yaw_90[4] = {sqrt_half, 0.0f, 0.0f, sqrt_half};
  assert(AttitudeEstimator_Init(&state, identity, bias, 1000000U));
  assert(AttitudeEstimator_Update(&state, 0U, bias) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 1000000U, biased_yaw) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, identity);
  assert(AttitudeEstimator_Update(&state, 2000000U, bias) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_90);
}

static void test_gap_requires_reinitialization(void)
{
  AttitudeEstimator state = {0};
  const float yaw_90[4] = {sqrt_half, 0.0f, 0.0f, sqrt_half};
  assert(AttitudeEstimator_Update(&state, 0U, stationary) == ATTITUDE_ESTIMATOR_NEEDS_INIT);
  assert(AttitudeEstimator_Init(&state, identity, stationary, 1000000U));
  assert(AttitudeEstimator_Update(&state, 0U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 1000000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  assert(AttitudeEstimator_Update(&state, 2000001U, stationary) == ATTITUDE_ESTIMATOR_DISCONTINUITY);
  assert(AttitudeEstimator_Update(&state, 2005001U, stationary) == ATTITUDE_ESTIMATOR_NEEDS_INIT);
  expect_attitude(&state, yaw_90);

  assert(AttitudeEstimator_Init(&state, state.q_body_to_ned, stationary, 5000U));
  assert(AttitudeEstimator_Update(&state, 2010001U, stationary) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(AttitudeEstimator_Update(&state, 2015001U, stationary) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_90);
}

static void test_invalid_inputs(void)
{
  AttitudeEstimator state = {0};
  const float zero_q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  const float nonfinite_q[4] = {1.0f, INFINITY, 0.0f, 0.0f};
  const float nonfinite_gyro[3] = {0.0f, NAN, 0.0f};
  const float yaw_90[4] = {sqrt_half, 0.0f, 0.0f, sqrt_half};
  assert(AttitudeEstimator_Init(&state, identity, stationary, 1000000U));
  assert(AttitudeEstimator_Update(&state, 0U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_PRIMED);
  assert(!AttitudeEstimator_Init(&state, zero_q, stationary, 1000000U));
  assert(!AttitudeEstimator_Init(&state, nonfinite_q, stationary, 1000000U));
  assert(!AttitudeEstimator_Init(&state, identity, nonfinite_gyro, 1000000U));
  assert(!AttitudeEstimator_Init(&state, identity, stationary, 0U));
  assert(AttitudeEstimator_Update(&state, 500000U, nonfinite_gyro) == ATTITUDE_ESTIMATOR_INVALID_INPUT);
  expect_attitude(&state, identity);
  assert(AttitudeEstimator_Update(&state, 1000000U, positive_yaw_rate) == ATTITUDE_ESTIMATOR_UPDATED);
  expect_attitude(&state, yaw_90);
}

static void expect_accel_mag_attitude(const float accel[3], const float mag[3],
                                      const float expected[4])
{
  float q[4];
  assert(AttitudeEstimator_QuaternionFromAccelMag(accel, mag, q));
  float dot = 0.0f;
  for (size_t index = 0U; index < 4U; ++index)
  {
    dot += q[index] * expected[index];
  }
  float sign = dot < 0.0f ? -1.0f : 1.0f;
  for (size_t index = 0U; index < 4U; ++index)
  {
    assert(fabsf(q[index] - sign * expected[index]) < 2.0e-5f);
  }
}

static void test_accel_mag_level_headings(void)
{
  const float accel[3] = {0.0f, 0.0f, -gravity};
  const float mag_north[3] = {22.0f, 0.0f, 44.0f};
  const float mag_east[3] = {0.0f, -22.0f, 44.0f};
  const float mag_south[3] = {-22.0f, 0.0f, 44.0f};
  const float yaw_90[4] = {sqrt_half, 0.0f, 0.0f, sqrt_half};
  const float yaw_180[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  expect_accel_mag_attitude(accel, mag_north, identity);
  expect_accel_mag_attitude(accel, mag_east, yaw_90);
  expect_accel_mag_attitude(accel, mag_south, yaw_180);
}

static void test_accel_mag_tilted_attitudes(void)
{
  const float sin_roll = 0.3420201433f;
  const float cos_roll = 0.9396926208f;
  const float accel[3] = {0.0f, -gravity * sin_roll, -gravity * cos_roll};
  const float mag[3] = {22.0f, 44.0f * sin_roll, 44.0f * cos_roll};
  const float roll_20[4] = {0.9848077530f, 0.1736481777f, 0.0f, 0.0f};
  expect_accel_mag_attitude(accel, mag, roll_20);

  const float accel_inverted[3] = {0.0f, 0.0f, gravity};
  const float mag_roll_inverted[3] = {22.0f, 0.0f, -44.0f};
  const float mag_pitch_inverted[3] = {-22.0f, 0.0f, -44.0f};
  const float roll_180[4] = {0.0f, 1.0f, 0.0f, 0.0f};
  const float pitch_180[4] = {0.0f, 0.0f, 1.0f, 0.0f};
  expect_accel_mag_attitude(accel_inverted, mag_roll_inverted, roll_180);
  expect_accel_mag_attitude(accel_inverted, mag_pitch_inverted, pitch_180);
}

static void test_accel_mag_rejects_unusable_vectors(void)
{
  float q[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  const float accel[3] = {0.0f, 0.0f, -gravity};
  const float mag[3] = {22.0f, 0.0f, 44.0f};
  const float vertical_mag[3] = {0.0f, 0.0f, 44.0f};
  const float nearly_vertical_mag[3] = {0.01f, 0.0f, -44.0f};
  const float invalid[3] = {NAN, 0.0f, INFINITY};

  assert(!AttitudeEstimator_QuaternionFromAccelMag(stationary, mag, q));
  assert(!AttitudeEstimator_QuaternionFromAccelMag(accel, stationary, q));
  assert(!AttitudeEstimator_QuaternionFromAccelMag(accel, vertical_mag, q));
  assert(!AttitudeEstimator_QuaternionFromAccelMag(accel, nearly_vertical_mag, q));
  assert(!AttitudeEstimator_QuaternionFromAccelMag(invalid, mag, q));
  assert(!AttitudeEstimator_QuaternionFromAccelMag(accel, invalid, q));
  for (size_t index = 0U; index < 4U; ++index)
  {
    assert(q[index] == (float)(index + 1U));
  }
}

int main(void)
{
  test_stationary_and_initial_normalization();
  test_yaw_and_body_side_multiplication();
  test_timestamps_and_previous_sample_hold();
  test_bias_subtraction();
  test_gap_requires_reinitialization();
  test_invalid_inputs();
  test_accel_mag_level_headings();
  test_accel_mag_tilted_attitudes();
  test_accel_mag_rejects_unusable_vectors();
  puts("9 attitude estimator tests passed");
  return 0;
}
