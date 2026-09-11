#ifndef TARZAN_ORIENTATION_H
#define TARZAN_ORIENTATION_H

#include <Tarzan/lib/madgwick.h>

#include <stdbool.h>

struct complementary_filter_state {
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float alpha;
  bool initialized;
};

struct orientation_estimator {
  struct madgwick_state madgwick;
  struct complementary_filter_state complementary;
};

/* Initialize both filters. complementary_alpha must be in [0, 1]. */
void orientation_estimator_init(struct orientation_estimator *estimator,
                                float madgwick_beta,
                                float complementary_alpha);

/* Six-axis update. Yaw is gyro-only and will drift without a magnetometer. */
bool orientation_update_imu(struct orientation_estimator *estimator,
                            float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float dt_seconds);

/* Nine-axis update. The magnetometer corrects yaw in both filters. */
bool orientation_update_marg(struct orientation_estimator *estimator,
                             float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float mx, float my, float mz,
                             float dt_seconds);

/* Madgwick Euler output in radians. */
bool orientation_get_euler_rad(const struct orientation_estimator *estimator,
                               float *roll_rad,
                               float *pitch_rad,
                               float *yaw_rad);

/* child orientation expressed in the parent sensor frame. */
bool quaternion_relative(const struct quaternion *parent,
                         const struct quaternion *child,
                         struct quaternion *relative);

/* Extract signed rotation about a unit-agnostic axis from a relative
 * quaternion. The result is wrapped to [-pi, pi]. */
bool quaternion_twist_angle(const struct quaternion *relative,
                            float axis_x, float axis_y, float axis_z,
                            float *angle_rad);

#endif /* TARZAN_ORIENTATION_H */
