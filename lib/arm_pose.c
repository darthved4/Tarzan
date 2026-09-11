#include <Tarzan/lib/arm_pose.h>

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>

#define PI_F 3.14159265358979323846f

static float wrap(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static int read_heading(const struct device *device, float *heading) {
  struct sensor_value value[3];
  int rc = sensor_sample_fetch(device);
  if (rc == 0) {
    rc = sensor_channel_get(device, SENSOR_CHAN_MAGN_XYZ, value);
  }
  if (rc != 0) {
    return rc;
  }
  const float x = (float)sensor_value_to_double(&value[0]);
  const float y = (float)sensor_value_to_double(&value[1]);
  if (!isfinite(x) || !isfinite(y) || (x == 0.0f && y == 0.0f)) {
    return -EIO;
  }
  *heading = atan2f(y, x);
  return 0;
}

static bool raw_angles(struct arm_pose_state *state,
                       float turntable_heading,
                       struct arm_joint_angles *madgwick,
                       struct arm_joint_angles *complementary) {
  float first_roll;
  float first_yaw;
  struct quaternion second_relative;
  struct quaternion wrist_relative;
  if (!orientation_get_euler_rad(&state->first_link.estimator, &first_roll,
                                 &madgwick->first_link, &first_yaw) ||
      !quaternion_relative(&state->first_link.estimator.madgwick.orientation,
                           &state->second_link.estimator.madgwick.orientation,
                           &second_relative) ||
      !quaternion_relative(&state->second_link.estimator.madgwick.orientation,
                           &state->end_effector.estimator.madgwick.orientation,
                           &wrist_relative) ||
      !quaternion_twist_angle(&second_relative, 0.0f, 1.0f, 0.0f,
                              &madgwick->second_link) ||
      !quaternion_twist_angle(&wrist_relative, 0.0f, 1.0f, 0.0f,
                              &madgwick->wrist_pitch) ||
      !quaternion_twist_angle(&wrist_relative, 1.0f, 0.0f, 0.0f,
                              &madgwick->wrist_roll)) {
    return false;
  }
  madgwick->turntable = turntable_heading;

  const struct complementary_filter_state *first =
      &state->first_link.estimator.complementary;
  const struct complementary_filter_state *second =
      &state->second_link.estimator.complementary;
  const struct complementary_filter_state *end =
      &state->end_effector.estimator.complementary;
  *complementary = (struct arm_joint_angles){
      .turntable = turntable_heading,
      .first_link = first->pitch_rad,
      .second_link = wrap(second->pitch_rad - first->pitch_rad),
      .wrist_pitch = wrap(end->pitch_rad - second->pitch_rad),
      .wrist_roll = wrap(end->roll_rad - second->roll_rad),
  };
  return true;
}

static struct arm_joint_angles subtract(struct arm_joint_angles value,
                                        const struct arm_joint_angles *zero) {
  value.turntable = wrap(value.turntable - zero->turntable);
  value.first_link = wrap(value.first_link - zero->first_link);
  value.second_link = wrap(value.second_link - zero->second_link);
  value.wrist_pitch = wrap(value.wrist_pitch - zero->wrist_pitch);
  value.wrist_roll = wrap(value.wrist_roll - zero->wrist_roll);
  return value;
}

int arm_pose_init(struct arm_pose_state *state,
                  const struct arm_pose_config *config) {
  if (state == NULL || config == NULL || config->turntable_mag == NULL ||
      config->base_reference_mag == NULL ||
      !device_is_ready(config->turntable_mag) ||
      !device_is_ready(config->base_reference_mag)) {
    return -EINVAL;
  }
  memset(state, 0, sizeof(*state));
  state->turntable_mag = config->turntable_mag;
  state->base_reference_mag = config->base_reference_mag;

  int rc = link_orientation_init(
      &state->first_link, config->first_link_imu, NULL, false,
      &config->first_link_map, &SENSOR_AXIS_MAP_IDENTITY,
      config->madgwick_beta, config->complementary_alpha);
  if (rc == 0) {
    rc = link_orientation_init(
        &state->second_link, config->second_link_imu, NULL, false,
        &config->second_link_map, &SENSOR_AXIS_MAP_IDENTITY,
        config->madgwick_beta, config->complementary_alpha);
  }
  if (rc == 0) {
    rc = link_orientation_init(
        &state->end_effector, config->end_effector_imu,
        config->end_effector_mag, true, &config->end_effector_imu_map,
        &config->end_effector_mag_map, config->madgwick_beta,
        config->complementary_alpha);
  }
  return rc;
}

int arm_pose_calibrate_gyros(struct arm_pose_state *state, uint16_t samples,
                             uint16_t sample_delay_ms) {
  if (state == NULL) {
    return -EINVAL;
  }
  int rc = link_orientation_calibrate_gyro(&state->first_link, samples,
                                           sample_delay_ms);
  if (rc == 0) {
    rc = link_orientation_calibrate_gyro(&state->second_link, samples,
                                         sample_delay_ms);
  }
  if (rc == 0) {
    rc = link_orientation_calibrate_gyro(&state->end_effector, samples,
                                         sample_delay_ms);
  }
  return rc;
}

int arm_pose_update(struct arm_pose_state *state, float dt_seconds) {
  if (state == NULL) {
    return -EINVAL;
  }
  float turntable;
  float reference;
  int rc = read_heading(state->turntable_mag, &turntable);
  if (rc == 0) {
    rc = read_heading(state->base_reference_mag, &reference);
  }
  if (rc == 0) {
    rc = link_orientation_update(&state->first_link, dt_seconds);
  }
  if (rc == 0) {
    rc = link_orientation_update(&state->second_link, dt_seconds);
  }
  if (rc == 0) {
    rc = link_orientation_update(&state->end_effector, dt_seconds);
  }
  if (rc != 0) {
    return rc;
  }

  struct arm_joint_angles madgwick;
  struct arm_joint_angles complementary;
  if (!raw_angles(state, wrap(turntable - reference), &madgwick,
                  &complementary)) {
    return -EIO;
  }
  if (!state->zeroed) {
    state->madgwick_zero = madgwick;
    state->complementary_zero = complementary;
    state->zeroed = true;
  }
  state->madgwick = subtract(madgwick, &state->madgwick_zero);
  state->complementary = subtract(complementary,
                                  &state->complementary_zero);
  return 0;
}

void arm_pose_zero(struct arm_pose_state *state) {
  if (state == NULL || !state->zeroed) {
    return;
  }
  state->madgwick_zero.turntable =
      wrap(state->madgwick_zero.turntable + state->madgwick.turntable);
  state->madgwick_zero.first_link =
      wrap(state->madgwick_zero.first_link + state->madgwick.first_link);
  state->madgwick_zero.second_link =
      wrap(state->madgwick_zero.second_link + state->madgwick.second_link);
  state->madgwick_zero.wrist_pitch =
      wrap(state->madgwick_zero.wrist_pitch + state->madgwick.wrist_pitch);
  state->madgwick_zero.wrist_roll =
      wrap(state->madgwick_zero.wrist_roll + state->madgwick.wrist_roll);
  state->complementary_zero.turntable = wrap(
      state->complementary_zero.turntable + state->complementary.turntable);
  state->complementary_zero.first_link = wrap(
      state->complementary_zero.first_link + state->complementary.first_link);
  state->complementary_zero.second_link = wrap(
      state->complementary_zero.second_link + state->complementary.second_link);
  state->complementary_zero.wrist_pitch = wrap(
      state->complementary_zero.wrist_pitch + state->complementary.wrist_pitch);
  state->complementary_zero.wrist_roll = wrap(
      state->complementary_zero.wrist_roll + state->complementary.wrist_roll);
  state->madgwick = (struct arm_joint_angles){0};
  state->complementary = (struct arm_joint_angles){0};
}
