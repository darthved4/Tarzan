#include <Tarzan/lib/madgwick.h>

#include <math.h>
#include <stddef.h>

#define MADGWICK_MAX_DT_SECONDS 0.5f
#define MADGWICK_MIN_NORM_SQUARED 1.0e-20f
#define RAD_TO_DEG 57.29577951308232f

static bool finite_vector3(float x, float y, float z) {
  return isfinite(x) && isfinite(y) && isfinite(z);
}

static float inverse_sqrt(float value) {
  if (!isfinite(value) || value <= MADGWICK_MIN_NORM_SQUARED) {
    return 0.0f;
  }
  return 1.0f / sqrtf(value);
}

static bool state_is_valid(const struct madgwick_state *state) {
  if (state == NULL || !isfinite(state->beta) || state->beta < 0.0f) {
    return false;
  }

  const struct quaternion *q = &state->orientation;
  return isfinite(q->w) && isfinite(q->x) && isfinite(q->y) &&
         isfinite(q->z);
}

static bool commit_orientation(struct madgwick_state *state,
                               float w, float x, float y, float z) {
  const float inverse_norm =
      inverse_sqrt(w * w + x * x + y * y + z * z);
  if (inverse_norm == 0.0f) {
    return false;
  }

  state->orientation.w = w * inverse_norm;
  state->orientation.x = x * inverse_norm;
  state->orientation.y = y * inverse_norm;
  state->orientation.z = z * inverse_norm;
  return true;
}

void madgwick_init(struct madgwick_state *state, float beta) {
  if (state == NULL) {
    return;
  }

  state->orientation =
      (struct quaternion){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
  state->beta =
      isfinite(beta) && beta >= 0.0f ? beta : MADGWICK_BETA_DEFAULT;
}

bool madgwick_update_imu(struct madgwick_state *state,
                         float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float dt_seconds) {
  if (!state_is_valid(state) || !finite_vector3(ax, ay, az) ||
      !finite_vector3(gx, gy, gz) || !isfinite(dt_seconds) ||
      dt_seconds <= 0.0f || dt_seconds > MADGWICK_MAX_DT_SECONDS) {
    return false;
  }

  float inverse_norm = inverse_sqrt(ax * ax + ay * ay + az * az);
  if (inverse_norm == 0.0f) {
    return false;
  }

  ax *= inverse_norm;
  ay *= inverse_norm;
  az *= inverse_norm;

  const float w = state->orientation.w;
  const float x = state->orientation.x;
  const float y = state->orientation.y;
  const float z = state->orientation.z;

  const float f1 = 2.0f * (x * z - w * y) - ax;
  const float f2 = 2.0f * (w * x + y * z) - ay;
  const float f3 = 2.0f * (0.5f - x * x - y * y) - az;

  float s1 = -2.0f * y * f1 + 2.0f * x * f2;
  float s2 = 2.0f * z * f1 + 2.0f * w * f2 - 4.0f * x * f3;
  float s3 = -2.0f * w * f1 + 2.0f * z * f2 - 4.0f * y * f3;
  float s4 = 2.0f * x * f1 + 2.0f * y * f2;

  inverse_norm = inverse_sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  if (inverse_norm != 0.0f) {
    s1 *= inverse_norm;
    s2 *= inverse_norm;
    s3 *= inverse_norm;
    s4 *= inverse_norm;
  } else {
    s1 = 0.0f;
    s2 = 0.0f;
    s3 = 0.0f;
    s4 = 0.0f;
  }

  const float q_dot_w =
      0.5f * (-x * gx - y * gy - z * gz) - state->beta * s1;
  const float q_dot_x =
      0.5f * (w * gx + y * gz - z * gy) - state->beta * s2;
  const float q_dot_y =
      0.5f * (w * gy - x * gz + z * gx) - state->beta * s3;
  const float q_dot_z =
      0.5f * (w * gz + x * gy - y * gx) - state->beta * s4;

  return commit_orientation(state,
                            w + q_dot_w * dt_seconds,
                            x + q_dot_x * dt_seconds,
                            y + q_dot_y * dt_seconds,
                            z + q_dot_z * dt_seconds);
}

bool madgwick_update(struct madgwick_state *state,
                     float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float mx, float my, float mz,
                     float dt_seconds) {
  if (!state_is_valid(state) || !finite_vector3(ax, ay, az) ||
      !finite_vector3(gx, gy, gz) || !finite_vector3(mx, my, mz) ||
      !isfinite(dt_seconds) || dt_seconds <= 0.0f ||
      dt_seconds > MADGWICK_MAX_DT_SECONDS) {
    return false;
  }

  float inverse_norm = inverse_sqrt(ax * ax + ay * ay + az * az);
  if (inverse_norm == 0.0f) {
    return false;
  }

  const float magnetic_norm_squared = mx * mx + my * my + mz * mz;
  if (inverse_sqrt(magnetic_norm_squared) == 0.0f) {
    return madgwick_update_imu(state, ax, ay, az, gx, gy, gz,
                               dt_seconds);
  }

  ax *= inverse_norm;
  ay *= inverse_norm;
  az *= inverse_norm;

  inverse_norm = inverse_sqrt(magnetic_norm_squared);
  mx *= inverse_norm;
  my *= inverse_norm;
  mz *= inverse_norm;

  const float w = state->orientation.w;
  const float x = state->orientation.x;
  const float y = state->orientation.y;
  const float z = state->orientation.z;

  const float hx =
      2.0f * mx * (0.5f - y * y - z * z) +
      2.0f * my * (x * y - w * z) +
      2.0f * mz * (x * z + w * y);
  const float hy =
      2.0f * mx * (x * y + w * z) +
      2.0f * my * (0.5f - x * x - z * z) +
      2.0f * mz * (y * z - w * x);
  const float bx = sqrtf(hx * hx + hy * hy);
  const float bz =
      2.0f * mx * (x * z - w * y) +
      2.0f * my * (y * z + w * x) +
      2.0f * mz * (0.5f - x * x - y * y);

  const float f1 = 2.0f * (x * z - w * y) - ax;
  const float f2 = 2.0f * (w * x + y * z) - ay;
  const float f3 = 2.0f * (0.5f - x * x - y * y) - az;
  const float f4 =
      2.0f * bx * (0.5f - y * y - z * z) +
      2.0f * bz * (x * z - w * y) - mx;
  const float f5 =
      2.0f * bx * (x * y - w * z) +
      2.0f * bz * (w * x + y * z) - my;
  const float f6 =
      2.0f * bx * (w * y + x * z) +
      2.0f * bz * (0.5f - x * x - y * y) - mz;

  float s1 =
      -2.0f * y * f1 + 2.0f * x * f2 -
      2.0f * bz * y * f4 +
      (-2.0f * bx * z + 2.0f * bz * x) * f5 +
      2.0f * bx * y * f6;
  float s2 =
      2.0f * z * f1 + 2.0f * w * f2 - 4.0f * x * f3 +
      2.0f * bz * z * f4 +
      (2.0f * bx * y + 2.0f * bz * w) * f5 +
      (2.0f * bx * z - 4.0f * bz * x) * f6;
  float s3 =
      -2.0f * w * f1 + 2.0f * z * f2 - 4.0f * y * f3 +
      (-4.0f * bx * y - 2.0f * bz * w) * f4 +
      (2.0f * bx * x + 2.0f * bz * z) * f5 +
      (2.0f * bx * w - 4.0f * bz * y) * f6;
  float s4 =
      2.0f * x * f1 + 2.0f * y * f2 +
      (-4.0f * bx * z + 2.0f * bz * x) * f4 +
      (-2.0f * bx * w + 2.0f * bz * y) * f5 +
      2.0f * bx * x * f6;

  inverse_norm = inverse_sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
  if (inverse_norm != 0.0f) {
    s1 *= inverse_norm;
    s2 *= inverse_norm;
    s3 *= inverse_norm;
    s4 *= inverse_norm;
  } else {
    s1 = 0.0f;
    s2 = 0.0f;
    s3 = 0.0f;
    s4 = 0.0f;
  }

  const float q_dot_w =
      0.5f * (-x * gx - y * gy - z * gz) - state->beta * s1;
  const float q_dot_x =
      0.5f * (w * gx + y * gz - z * gy) - state->beta * s2;
  const float q_dot_y =
      0.5f * (w * gy - x * gz + z * gx) - state->beta * s3;
  const float q_dot_z =
      0.5f * (w * gz + x * gy - y * gx) - state->beta * s4;

  return commit_orientation(state,
                            w + q_dot_w * dt_seconds,
                            x + q_dot_x * dt_seconds,
                            y + q_dot_y * dt_seconds,
                            z + q_dot_z * dt_seconds);
}

void madgwick_get_euler(const struct quaternion *orientation,
                        float *roll, float *pitch, float *yaw) {
  if (orientation == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
    return;
  }

  const float w = orientation->w;
  const float x = orientation->x;
  const float y = orientation->y;
  const float z = orientation->z;

  *roll = atan2f(2.0f * (w * x + y * z),
                 1.0f - 2.0f * (x * x + y * y)) *
          RAD_TO_DEG;

  const float sin_pitch = 2.0f * (w * y - z * x);
  if (fabsf(sin_pitch) >= 1.0f) {
    *pitch = copysignf(90.0f, sin_pitch);
  } else {
    *pitch = asinf(sin_pitch) * RAD_TO_DEG;
  }

  *yaw = atan2f(2.0f * (w * z + x * y),
                1.0f - 2.0f * (y * y + z * z)) *
         RAD_TO_DEG;
}
