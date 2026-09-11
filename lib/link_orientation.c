#include <Tarzan/lib/link_orientation.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

const struct sensor_axis_map SENSOR_AXIS_MAP_IDENTITY = {
    .source_axis = {0U, 1U, 2U},
    .sign = {1, 1, 1},
};

static bool axis_map_valid(const struct sensor_axis_map *map) {
  if (map == NULL) {
    return false;
  }

  bool used[3] = {false, false, false};
  for (int axis = 0; axis < 3; axis++) {
    if (map->source_axis[axis] > 2U || used[map->source_axis[axis]] ||
        (map->sign[axis] != -1 && map->sign[axis] != 1)) {
      return false;
    }
    used[map->source_axis[axis]] = true;
  }
  return true;
}

static void apply_axis_map(const struct sensor_axis_map *map,
                           const float input[3], float output[3]) {
  for (int axis = 0; axis < 3; axis++) {
    output[axis] = (float)map->sign[axis] * input[map->source_axis[axis]];
  }
}

static int fetch_vector(const struct device *device,
                        enum sensor_channel channel, float output[3]) {
  struct sensor_value values[3];
  int rc = sensor_sample_fetch(device);
  if (rc != 0) {
    return rc;
  }
  rc = sensor_channel_get(device, channel, values);
  if (rc != 0) {
    return rc;
  }
  for (int axis = 0; axis < 3; axis++) {
    output[axis] = (float)sensor_value_to_double(&values[axis]);
  }
  return 0;
}

int link_orientation_init(struct link_orientation *link,
                          const struct device *imu,
                          const struct device *mag,
                          bool use_magnetometer,
                          const struct sensor_axis_map *imu_map,
                          const struct sensor_axis_map *mag_map,
                          float madgwick_beta,
                          float complementary_alpha) {
  if (link == NULL || imu == NULL || !device_is_ready(imu) ||
      (use_magnetometer &&
       (mag == NULL || !device_is_ready(mag))) ||
      !axis_map_valid(imu_map) || !axis_map_valid(mag_map)) {
    return -EINVAL;
  }

  memset(link, 0, sizeof(*link));
  link->imu = imu;
  link->mag = mag;
  link->use_magnetometer = use_magnetometer;
  link->imu_map = *imu_map;
  link->mag_map = *mag_map;
  for (int axis = 0; axis < 3; axis++) {
    link->mag_scale[axis] = 1.0f;
  }
  orientation_estimator_init(&link->estimator, madgwick_beta,
                             complementary_alpha);
  return 0;
}

int link_orientation_calibrate_gyro(struct link_orientation *link,
                                    uint16_t samples,
                                    uint16_t sample_delay_ms) {
  if (link == NULL || link->imu == NULL || samples == 0U) {
    return -EINVAL;
  }

  float sum[3] = {0.0f, 0.0f, 0.0f};
  for (uint16_t sample = 0U; sample < samples; sample++) {
    float raw[3];
    float mapped[3];
    const int rc = fetch_vector(link->imu, SENSOR_CHAN_GYRO_XYZ, raw);
    if (rc != 0) {
      return rc;
    }
    apply_axis_map(&link->imu_map, raw, mapped);
    for (int axis = 0; axis < 3; axis++) {
      sum[axis] += mapped[axis];
    }
    if (sample_delay_ms != 0U) {
      k_sleep(K_MSEC(sample_delay_ms));
    }
  }

  for (int axis = 0; axis < 3; axis++) {
    link->gyro_offset[axis] = sum[axis] / (float)samples;
  }
  return 0;
}

int link_orientation_calibrate_magnetometer(struct link_orientation *link,
                                             uint16_t samples,
                                             uint16_t sample_delay_ms) {
  if (link == NULL || !link->use_magnetometer || link->mag == NULL ||
      samples == 0U) {
    return -EINVAL;
  }

  float minimum[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float maximum[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  for (uint16_t sample = 0U; sample < samples; sample++) {
    float raw[3];
    float mapped[3];
    const int rc = fetch_vector(link->mag, SENSOR_CHAN_MAGN_XYZ, raw);
    if (rc != 0) {
      return rc;
    }
    apply_axis_map(&link->mag_map, raw, mapped);
    for (int axis = 0; axis < 3; axis++) {
      minimum[axis] = fminf(minimum[axis], mapped[axis]);
      maximum[axis] = fmaxf(maximum[axis], mapped[axis]);
    }
    if (sample_delay_ms != 0U) {
      k_sleep(K_MSEC(sample_delay_ms));
    }
  }

  float radius[3];
  for (int axis = 0; axis < 3; axis++) {
    link->mag_offset[axis] = 0.5f * (maximum[axis] + minimum[axis]);
    radius[axis] = 0.5f * (maximum[axis] - minimum[axis]);
    if (!isfinite(radius[axis]) || radius[axis] <= 1.0e-6f) {
      return -ERANGE;
    }
  }

  const float average_radius =
      (radius[0] + radius[1] + radius[2]) / 3.0f;
  for (int axis = 0; axis < 3; axis++) {
    link->mag_scale[axis] = average_radius / radius[axis];
  }
  return 0;
}

int link_orientation_update(struct link_orientation *link, float dt_seconds) {
  if (link == NULL || link->imu == NULL || !isfinite(dt_seconds) ||
      dt_seconds <= 0.0f) {
    return -EINVAL;
  }

  float accel_raw[3];
  float gyro_raw[3];
  float accel[3];
  float gyro[3];
  int rc = sensor_sample_fetch(link->imu);
  if (rc != 0) {
    return rc;
  }

  struct sensor_value accel_values[3];
  struct sensor_value gyro_values[3];
  rc = sensor_channel_get(link->imu, SENSOR_CHAN_ACCEL_XYZ, accel_values);
  if (rc != 0) {
    return rc;
  }
  rc = sensor_channel_get(link->imu, SENSOR_CHAN_GYRO_XYZ, gyro_values);
  if (rc != 0) {
    return rc;
  }
  for (int axis = 0; axis < 3; axis++) {
    accel_raw[axis] = (float)sensor_value_to_double(&accel_values[axis]);
    gyro_raw[axis] = (float)sensor_value_to_double(&gyro_values[axis]);
  }
  apply_axis_map(&link->imu_map, accel_raw, accel);
  apply_axis_map(&link->imu_map, gyro_raw, gyro);
  for (int axis = 0; axis < 3; axis++) {
    gyro[axis] -= link->gyro_offset[axis];
  }

  if (!link->use_magnetometer) {
    return orientation_update_imu(&link->estimator, accel[0], accel[1],
                                  accel[2], gyro[0], gyro[1], gyro[2],
                                  dt_seconds)
               ? 0
               : -EIO;
  }

  float mag_raw[3];
  float mag[3];
  rc = fetch_vector(link->mag, SENSOR_CHAN_MAGN_XYZ, mag_raw);
  if (rc != 0) {
    return rc;
  }
  apply_axis_map(&link->mag_map, mag_raw, mag);
  for (int axis = 0; axis < 3; axis++) {
    mag[axis] = (mag[axis] - link->mag_offset[axis]) * link->mag_scale[axis];
  }
  return orientation_update_marg(&link->estimator, accel[0], accel[1],
                                 accel[2], gyro[0], gyro[1], gyro[2], mag[0],
                                 mag[1], mag[2], dt_seconds)
             ? 0
             : -EIO;
}
