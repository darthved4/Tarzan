#ifndef TARZAN_LINK_ORIENTATION_H
#define TARZAN_LINK_ORIENTATION_H

#include <Tarzan/lib/orientation.h>

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/* Signed permutation from sensor axes into the link/body frame.
 * source_axis values are 0=X, 1=Y, 2=Z; sign values must be -1 or +1. */
struct sensor_axis_map {
  uint8_t source_axis[3];
  int8_t sign[3];
};

struct link_orientation {
  const struct device *imu;
  const struct device *mag;
  struct orientation_estimator estimator;
  struct sensor_axis_map imu_map;
  struct sensor_axis_map mag_map;
  float gyro_offset[3];
  float mag_offset[3];
  float mag_scale[3];
  bool use_magnetometer;
};

extern const struct sensor_axis_map SENSOR_AXIS_MAP_IDENTITY;

/* mag may be NULL only when use_magnetometer is false. */
int link_orientation_init(struct link_orientation *link,
                          const struct device *imu,
                          const struct device *mag,
                          bool use_magnetometer,
                          const struct sensor_axis_map *imu_map,
                          const struct sensor_axis_map *mag_map,
                          float madgwick_beta,
                          float complementary_alpha);

/* Keep the link stationary during gyro calibration. */
int link_orientation_calibrate_gyro(struct link_orientation *link,
                                    uint16_t samples,
                                    uint16_t sample_delay_ms);

/* Move the magnetometer through all orientations during this calibration. */
int link_orientation_calibrate_magnetometer(struct link_orientation *link,
                                             uint16_t samples,
                                             uint16_t sample_delay_ms);

/* Fetch one sensor sample and advance both filters. */
int link_orientation_update(struct link_orientation *link, float dt_seconds);

#endif /* TARZAN_LINK_ORIENTATION_H */
