#ifndef TARZAN_MADGWICK_H
#define TARZAN_MADGWICK_H

#include <stdbool.h>

#define MADGWICK_BETA_DEFAULT 0.3f

struct quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct madgwick_state {
  struct quaternion orientation;
  float beta;
};

/* Initialize an independent filter instance. A negative or non-finite beta
 * selects MADGWICK_BETA_DEFAULT. */
void madgwick_init(struct madgwick_state *state, float beta);

/* Update orientation from accelerometer, gyroscope, and magnetometer data.
 * Accelerometer units may be any consistent linear-acceleration unit,
 * gyroscope inputs are rad/s, and dt_seconds is seconds. Invalid
 * magnetometer data automatically selects the 6-DoF update. */
bool madgwick_update(struct madgwick_state *state,
                     float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float mx, float my, float mz,
                     float dt_seconds);

/* Update orientation from accelerometer and gyroscope data only. */
bool madgwick_update_imu(struct madgwick_state *state,
                         float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float dt_seconds);

/* Convert a quaternion to ZYX roll, pitch, and yaw angles in degrees. */
void madgwick_get_euler(const struct quaternion *orientation,
                        float *roll, float *pitch, float *yaw);

#endif /* TARZAN_MADGWICK_H */
