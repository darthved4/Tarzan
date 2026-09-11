#include <Tarzan/lib/orientation.h>

#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)
#define MIN_NORM_SQUARED 1.0e-20f

static bool finite_vector3(float x, float y, float z) {
  return isfinite(x) && isfinite(y) && isfinite(z);
}

static float wrap_radians(float angle) {
  while (angle > PI_F) {
    angle -= TWO_PI_F;
  }
  while (angle < -PI_F) {
    angle += TWO_PI_F;
  }
  return angle;
}

static bool complementary_update_imu(struct complementary_filter_state *state,
                                     float ax, float ay, float az,
                                     float gx, float gy, float gz,
                                     float dt_seconds) {
  if (state == NULL || !finite_vector3(ax, ay, az) ||
      !finite_vector3(gx, gy, gz) || !isfinite(dt_seconds) ||
      dt_seconds <= 0.0f || !isfinite(state->alpha) || state->alpha < 0.0f ||
      state->alpha > 1.0f) {
    return false;
  }

  const float accel_norm_squared = ax * ax + ay * ay + az * az;
  if (!isfinite(accel_norm_squared) ||
      accel_norm_squared <= MIN_NORM_SQUARED) {
    return false;
  }

  const float roll_acc = atan2f(ay, az);
  const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

  if (!state->initialized) {
    state->roll_rad = roll_acc;
    state->pitch_rad = pitch_acc;
    state->yaw_rad = 0.0f;
    state->initialized = true;
  }

  state->roll_rad = state->alpha * (state->roll_rad + gx * dt_seconds) +
                    (1.0f - state->alpha) * roll_acc;
  state->pitch_rad = state->alpha * (state->pitch_rad + gy * dt_seconds) +
                     (1.0f - state->alpha) * pitch_acc;
  state->yaw_rad = wrap_radians(state->yaw_rad + gz * dt_seconds);
  return isfinite(state->roll_rad) && isfinite(state->pitch_rad) &&
         isfinite(state->yaw_rad);
}

static bool complementary_update_marg(
    struct complementary_filter_state *state,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float dt_seconds) {
  if (!finite_vector3(mx, my, mz) ||
      !complementary_update_imu(state, ax, ay, az, gx, gy, gz, dt_seconds)) {
    return false;
  }

  const float mag_norm_squared = mx * mx + my * my + mz * mz;
  if (!isfinite(mag_norm_squared) || mag_norm_squared <= MIN_NORM_SQUARED) {
    return true; /* Retain gyro-only yaw when magnetic data is unavailable. */
  }

  const float sin_roll = sinf(state->roll_rad);
  const float cos_roll = cosf(state->roll_rad);
  const float sin_pitch = sinf(state->pitch_rad);
  const float cos_pitch = cosf(state->pitch_rad);

  const float horizontal_x = mx * cos_pitch + mz * sin_pitch;
  const float horizontal_y = mx * sin_roll * sin_pitch + my * cos_roll -
                             mz * sin_roll * cos_pitch;
  const float yaw_mag = atan2f(-horizontal_y, horizontal_x);
  const float correction = wrap_radians(yaw_mag - state->yaw_rad);
  state->yaw_rad =
      wrap_radians(state->yaw_rad + (1.0f - state->alpha) * correction);
  return isfinite(state->yaw_rad);
}

void orientation_estimator_init(struct orientation_estimator *estimator,
                                float madgwick_beta,
                                float complementary_alpha) {
  if (estimator == NULL) {
    return;
  }

  madgwick_init(&estimator->madgwick, madgwick_beta);
  estimator->complementary = (struct complementary_filter_state){0};
  estimator->complementary.alpha =
      isfinite(complementary_alpha) && complementary_alpha >= 0.0f &&
              complementary_alpha <= 1.0f
          ? complementary_alpha
          : 0.98f;
}

bool orientation_update_imu(struct orientation_estimator *estimator,
                            float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float dt_seconds) {
  if (estimator == NULL) {
    return false;
  }

  const bool madgwick_ok = madgwick_update_imu(
      &estimator->madgwick, ax, ay, az, gx, gy, gz, dt_seconds);
  const bool complementary_ok = complementary_update_imu(
      &estimator->complementary, ax, ay, az, gx, gy, gz, dt_seconds);
  return madgwick_ok && complementary_ok;
}

bool orientation_update_marg(struct orientation_estimator *estimator,
                             float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float mx, float my, float mz,
                             float dt_seconds) {
  if (estimator == NULL) {
    return false;
  }

  const bool madgwick_ok = madgwick_update(
      &estimator->madgwick, ax, ay, az, gx, gy, gz, mx, my, mz, dt_seconds);
  const bool complementary_ok = complementary_update_marg(
      &estimator->complementary, ax, ay, az, gx, gy, gz, mx, my, mz,
      dt_seconds);
  return madgwick_ok && complementary_ok;
}

bool orientation_get_euler_rad(const struct orientation_estimator *estimator,
                               float *roll_rad,
                               float *pitch_rad,
                               float *yaw_rad) {
  if (estimator == NULL || roll_rad == NULL || pitch_rad == NULL ||
      yaw_rad == NULL) {
    return false;
  }

  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  madgwick_get_euler(&estimator->madgwick.orientation, &roll_deg, &pitch_deg,
                     &yaw_deg);
  *roll_rad = roll_deg * DEG_TO_RAD;
  *pitch_rad = pitch_deg * DEG_TO_RAD;
  *yaw_rad = yaw_deg * DEG_TO_RAD;
  return isfinite(*roll_rad) && isfinite(*pitch_rad) && isfinite(*yaw_rad);
}

static bool normalized_quaternion(const struct quaternion *input,
                                  struct quaternion *output) {
  if (input == NULL || output == NULL || !isfinite(input->w) ||
      !isfinite(input->x) || !isfinite(input->y) || !isfinite(input->z)) {
    return false;
  }

  const float norm_squared = input->w * input->w + input->x * input->x +
                             input->y * input->y + input->z * input->z;
  if (!isfinite(norm_squared) || norm_squared <= MIN_NORM_SQUARED) {
    return false;
  }

  const float inverse_norm = 1.0f / sqrtf(norm_squared);
  output->w = input->w * inverse_norm;
  output->x = input->x * inverse_norm;
  output->y = input->y * inverse_norm;
  output->z = input->z * inverse_norm;
  return true;
}

bool quaternion_relative(const struct quaternion *parent,
                         const struct quaternion *child,
                         struct quaternion *relative) {
  struct quaternion p;
  struct quaternion c;
  if (!normalized_quaternion(parent, &p) ||
      !normalized_quaternion(child, &c) || relative == NULL) {
    return false;
  }

  /* conjugate(parent) * child */
  const struct quaternion result = {
      .w = p.w * c.w + p.x * c.x + p.y * c.y + p.z * c.z,
      .x = p.w * c.x - p.x * c.w - p.y * c.z + p.z * c.y,
      .y = p.w * c.y + p.x * c.z - p.y * c.w - p.z * c.x,
      .z = p.w * c.z - p.x * c.y + p.y * c.x - p.z * c.w,
  };
  return normalized_quaternion(&result, relative);
}

bool quaternion_twist_angle(const struct quaternion *relative,
                            float axis_x, float axis_y, float axis_z,
                            float *angle_rad) {
  struct quaternion q;
  if (!normalized_quaternion(relative, &q) || angle_rad == NULL ||
      !finite_vector3(axis_x, axis_y, axis_z)) {
    return false;
  }

  const float axis_norm_squared =
      axis_x * axis_x + axis_y * axis_y + axis_z * axis_z;
  if (!isfinite(axis_norm_squared) || axis_norm_squared <= MIN_NORM_SQUARED) {
    return false;
  }

  const float inverse_axis_norm = 1.0f / sqrtf(axis_norm_squared);
  axis_x *= inverse_axis_norm;
  axis_y *= inverse_axis_norm;
  axis_z *= inverse_axis_norm;

  const float projected_vector =
      q.x * axis_x + q.y * axis_y + q.z * axis_z;
  *angle_rad = wrap_radians(2.0f * atan2f(projected_vector, q.w));
  return isfinite(*angle_rad);
}
