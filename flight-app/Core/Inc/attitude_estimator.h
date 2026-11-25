#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ATTITUDE_ESTIMATOR_PRIMED,
  ATTITUDE_ESTIMATOR_UPDATED,
  ATTITUDE_ESTIMATOR_NONINCREASING_TIME,
  ATTITUDE_ESTIMATOR_DISCONTINUITY,
  ATTITUDE_ESTIMATOR_INVALID_INPUT,
  ATTITUDE_ESTIMATOR_NEEDS_INIT,
} AttitudeEstimatorResult;

/* Zero-initialize before first use, then call AttitudeEstimator_Init.
 * The caller can read q_body_to_ned as [w, x, y, z]. All other state belongs
 * to the estimator. A single caller owns the state.
 */
typedef struct
{
  float q_body_to_ned[4];
  float gyro_bias_rad_s[3];
  float previous_rate_rad_s[3];
  uint64_t previous_time_us;
  uint64_t max_gap_us;
  bool initialized;
  bool primed;
} AttitudeEstimator;

/* Computes a stationary [w, x, y, z] body-to-NED quaternion from body FRD
 * specific force and magnetic field. The caller must establish stationarity.
 * The horizontal magnetic field is assumed to point north.
 * Returns false for zero, nonfinite, or nearly collinear inputs and leaves
 * q_body_to_ned unchanged on failure.
 */
bool AttitudeEstimator_QuaternionFromAccelMag(const float accel_m_s2[3],
                                              const float mag_uT[3],
                                              float q_body_to_ned[4]);

/* Copies and normalizes initial_q, and copies the fixed bias in body axes.
 * Returns false and leaves the state unchanged for invalid inputs or a zero gap limit.
 */
bool AttitudeEstimator_Init(AttitudeEstimator *state, const float initial_q[4],
                            const float gyro_bias_rad_s[3], uint64_t max_gap_us);

/* The first valid sample primes the previous-rate hold. Each following sample
 * advances the attitude using the held, bias-corrected rate over the elapsed
 * interval. A gap above max_gap_us leaves the last attitude available and
 * requires reinitialization. Other rejected samples leave the state unchanged.
 */
AttitudeEstimatorResult AttitudeEstimator_Update(AttitudeEstimator *state,
                                                 uint64_t sample_time_us,
                                                 const float gyro_rad_s[3]);

#endif
