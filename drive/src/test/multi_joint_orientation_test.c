#include <Tarzan/lib/link_orientation.h>

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#define TURN_TABLE_MAG_NODE DT_NODELABEL(turntable_mag)
#define BASE_REFERENCE_MAG_NODE DT_NODELABEL(base_reference_mag)
#define FIRST_LINK_IMU_NODE DT_NODELABEL(first_link_imu)
#define SECOND_LINK_IMU_NODE DT_NODELABEL(second_link_imu)
#define END_EFFECTOR_IMU_NODE DT_NODELABEL(end_effector_imu)
#define END_EFFECTOR_MAG_NODE DT_NODELABEL(end_effector_mag)

#define SAMPLE_PERIOD_MS 10U
#define GYRO_CALIBRATION_SAMPLES 500U
#define GYRO_CALIBRATION_DELAY_MS 2U
#define WARMUP_SAMPLES 200U
#define RAD_TO_DEG 57.29577951308232088f
#define PI_F 3.14159265358979323846f

/* TODO(mechanical): replace these identity maps after recording the X/Y/Z
 * direction printed on every installed sensor PCB. */
static const struct sensor_axis_map first_link_imu_map = {
    .source_axis = {0U, 1U, 2U}, .sign = {1, 1, 1}};
static const struct sensor_axis_map second_link_imu_map = {
    .source_axis = {0U, 1U, 2U}, .sign = {1, 1, 1}};
static const struct sensor_axis_map end_effector_imu_map = {
    .source_axis = {0U, 1U, 2U}, .sign = {1, 1, 1}};
static const struct sensor_axis_map end_effector_mag_map = {
    .source_axis = {0U, 1U, 2U}, .sign = {1, 1, 1}};

static struct link_orientation first_link;
static struct link_orientation second_link;
static struct link_orientation end_effector;

struct joint_angles {
  float turntable;
  float first_link;
  float second_link;
  float wrist_pitch;
  float wrist_roll;
};

static struct joint_angles madgwick_zero;
static struct joint_angles complementary_zero;
static struct joint_angles latest_madgwick;
static struct joint_angles latest_complementary;
static bool orientation_ready;
static float turntable_heading_rad;
K_MUTEX_DEFINE(orientation_lock);

static const struct device *const turntable_mag =
    DEVICE_DT_GET(TURN_TABLE_MAG_NODE);
static const struct device *const base_reference_mag =
    DEVICE_DT_GET(BASE_REFERENCE_MAG_NODE);

static float wrap_radians(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static int initialize_link(struct link_orientation *link,
                           const struct device *imu,
                           const struct device *mag,
                           bool use_magnetometer,
                           const struct sensor_axis_map *imu_map,
                           const struct sensor_axis_map *mag_map,
                           const char *name) {
  int rc = link_orientation_init(link, imu, mag, use_magnetometer, imu_map,
                                 mag_map, MADGWICK_BETA_DEFAULT, 0.98f);
  if (rc != 0) {
    printk("orientation: %s sensor is not ready (%d)\n", name, rc);
    return rc;
  }

  printk("orientation: keep %s stationary for gyro calibration\n", name);
  rc = link_orientation_calibrate_gyro(
      link, GYRO_CALIBRATION_SAMPLES, GYRO_CALIBRATION_DELAY_MS);
  if (rc != 0) {
    printk("orientation: %s gyro calibration failed (%d)\n", name, rc);
  }
  return rc;
}

static int initialize_sensors(void) {
  if (!device_is_ready(turntable_mag) ||
      !device_is_ready(base_reference_mag)) {
    printk("orientation: turntable/reference magnetometer is not ready\n");
    return -ENODEV;
  }
  int rc = initialize_link(&first_link, DEVICE_DT_GET(FIRST_LINK_IMU_NODE),
                           NULL, false, &first_link_imu_map,
                           &SENSOR_AXIS_MAP_IDENTITY, "first-link");
  if (rc != 0) {
    return rc;
  }
  rc = initialize_link(&second_link, DEVICE_DT_GET(SECOND_LINK_IMU_NODE), NULL,
                       false, &second_link_imu_map, &SENSOR_AXIS_MAP_IDENTITY,
                       "second-link");
  if (rc != 0) {
    return rc;
  }
  return initialize_link(
      &end_effector, DEVICE_DT_GET(END_EFFECTOR_IMU_NODE),
      DEVICE_DT_GET(END_EFFECTOR_MAG_NODE), true, &end_effector_imu_map,
      &end_effector_mag_map, "end-effector");
}

static int read_heading(const struct device *magnetometer, float *heading) {
  struct sensor_value values[3];
  int rc = sensor_sample_fetch(magnetometer);
  if (rc == 0) {
    rc = sensor_channel_get(magnetometer, SENSOR_CHAN_MAGN_XYZ, values);
  }
  if (rc != 0) {
    return rc;
  }

  const float x = (float)sensor_value_to_double(&values[0]);
  const float y = (float)sensor_value_to_double(&values[1]);
  if (!isfinite(x) || !isfinite(y) || (x == 0.0f && y == 0.0f)) {
    return -EIO;
  }
  *heading = atan2f(y, x);
  return 0;
}

static int update_sensors(float dt_seconds) {
  float turntable_heading;
  float reference_heading;
  int rc = read_heading(turntable_mag, &turntable_heading);
  if (rc == 0) {
    rc = read_heading(base_reference_mag, &reference_heading);
  }
  if (rc == 0) {
    turntable_heading_rad = wrap_radians(turntable_heading - reference_heading);
  }
  if (rc == 0) {
    rc = link_orientation_update(&first_link, dt_seconds);
  }
  if (rc == 0) {
    rc = link_orientation_update(&second_link, dt_seconds);
  }
  if (rc == 0) {
    rc = link_orientation_update(&end_effector, dt_seconds);
  }
  return rc;
}

static bool extract_madgwick_angles(struct joint_angles *angles) {
  struct quaternion first_from_start;
  struct quaternion second_from_first;
  struct quaternion wrist_from_second;
  static struct quaternion first_at_start;
  static float turntable_at_start;
  static bool captured_start;

  if (!captured_start) {
    first_at_start = first_link.estimator.madgwick.orientation;
    turntable_at_start = turntable_heading_rad;
    captured_start = true;
  }

  angles->turntable =
      wrap_radians(turntable_heading_rad - turntable_at_start);
  return quaternion_relative(&first_at_start,
                             &first_link.estimator.madgwick.orientation,
                             &first_from_start) &&
         quaternion_relative(&first_link.estimator.madgwick.orientation,
                             &second_link.estimator.madgwick.orientation,
                             &second_from_first) &&
         quaternion_relative(&second_link.estimator.madgwick.orientation,
                             &end_effector.estimator.madgwick.orientation,
                             &wrist_from_second) &&
         /* TODO(mechanical): Y/Y/Y/X are provisional link/wrist axes. */
         quaternion_twist_angle(&first_from_start, 0.0f, 1.0f, 0.0f,
                                &angles->first_link) &&
         quaternion_twist_angle(&second_from_first, 0.0f, 1.0f, 0.0f,
                                &angles->second_link) &&
         quaternion_twist_angle(&wrist_from_second, 0.0f, 1.0f, 0.0f,
                                &angles->wrist_pitch) &&
         quaternion_twist_angle(&wrist_from_second, 1.0f, 0.0f, 0.0f,
                                &angles->wrist_roll);
}

static struct joint_angles extract_complementary_angles(void) {
  const struct complementary_filter_state *fl =
      &first_link.estimator.complementary;
  const struct complementary_filter_state *sl =
      &second_link.estimator.complementary;
  const struct complementary_filter_state *ee =
      &end_effector.estimator.complementary;

  return (struct joint_angles){
      .turntable = turntable_heading_rad,
      .first_link = fl->pitch_rad,
      .second_link = wrap_radians(sl->pitch_rad - fl->pitch_rad),
      .wrist_pitch = wrap_radians(ee->pitch_rad - sl->pitch_rad),
      .wrist_roll = wrap_radians(ee->roll_rad - sl->roll_rad),
  };
}

static struct joint_angles subtract_zero(struct joint_angles value,
                                         const struct joint_angles *zero) {
  value.turntable = wrap_radians(value.turntable - zero->turntable);
  value.first_link = wrap_radians(value.first_link - zero->first_link);
  value.second_link = wrap_radians(value.second_link - zero->second_link);
  value.wrist_pitch = wrap_radians(value.wrist_pitch - zero->wrist_pitch);
  value.wrist_roll = wrap_radians(value.wrist_roll - zero->wrist_roll);
  return value;
}

static int orientation_shell_command(const struct shell *shell, size_t argc,
                                     char **argv) {
  if (argc == 2U && strcmp(argv[1], "status") == 0) {
    k_mutex_lock(&orientation_lock, K_FOREVER);
    if (!orientation_ready) {
      k_mutex_unlock(&orientation_lock);
      shell_error(shell, "orientation stack is not ready");
      return -EAGAIN;
    }
    shell_print(shell, "Madgwick deg: TT %.2f FL %.2f SL %.2f WP %.2f WR %.2f",
                (double)(latest_madgwick.turntable * RAD_TO_DEG),
                (double)(latest_madgwick.first_link * RAD_TO_DEG),
                (double)(latest_madgwick.second_link * RAD_TO_DEG),
                (double)(latest_madgwick.wrist_pitch * RAD_TO_DEG),
                (double)(latest_madgwick.wrist_roll * RAD_TO_DEG));
    shell_print(shell, "Complementary deg: TT %.2f FL %.2f SL %.2f WP %.2f WR %.2f",
                (double)(latest_complementary.turntable * RAD_TO_DEG),
                (double)(latest_complementary.first_link * RAD_TO_DEG),
                (double)(latest_complementary.second_link * RAD_TO_DEG),
                (double)(latest_complementary.wrist_pitch * RAD_TO_DEG),
                (double)(latest_complementary.wrist_roll * RAD_TO_DEG));
    k_mutex_unlock(&orientation_lock);
    return 0;
  }

  shell_error(shell, "Usage: orientation status");
  return -EINVAL;
}

SHELL_CMD_REGISTER(orientation, NULL, "Read all link orientation estimates",
                   orientation_shell_command);

int main(void) {
  printk("\n=== Tarzan multi-link orientation bring-up ===\n");
  printk("Link filters plus relative-magnetometer turntable heading.\n");
  if (initialize_sensors() != 0) {
    printk("orientation: initialization failed; check overlay and wiring\n");
    return -ENODEV;
  }

  for (unsigned int sample = 0U; sample < WARMUP_SAMPLES; sample++) {
    if (update_sensors((float)SAMPLE_PERIOD_MS / 1000.0f) != 0) {
      printk("orientation: sensor warm-up failed\n");
      return -EIO;
    }
    k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
  }

  if (!extract_madgwick_angles(&madgwick_zero)) {
    printk("orientation: could not establish joint zero\n");
    return -EIO;
  }
  complementary_zero = extract_complementary_angles();
  k_mutex_lock(&orientation_lock, K_FOREVER);
  orientation_ready = true;
  k_mutex_unlock(&orientation_lock);
  printk("orientation: ready; startup pose is zero\n");
  printk("CSV: ms,TT_M,FL_M,SL_M,WP_M,WR_M,TT_C,FL_C,SL_C,WP_C,WR_C\n");

  int64_t previous_ms = k_uptime_get();
  unsigned int telemetry_divider = 0U;
  while (true) {
    k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
    const int64_t now_ms = k_uptime_get();
    const float dt_seconds = (float)(now_ms - previous_ms) / 1000.0f;
    previous_ms = now_ms;

    struct joint_angles madgwick_raw;
    if (update_sensors(dt_seconds) != 0 ||
        !extract_madgwick_angles(&madgwick_raw)) {
      printk("orientation: sample failed\n");
      continue;
    }
    k_mutex_lock(&orientation_lock, K_FOREVER);
    latest_madgwick = subtract_zero(madgwick_raw, &madgwick_zero);
    latest_complementary = subtract_zero(extract_complementary_angles(),
                                         &complementary_zero);

    if (++telemetry_divider >= 10U) {
      telemetry_divider = 0U;
      printk("ORI,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             now_ms, (double)(latest_madgwick.turntable * RAD_TO_DEG),
             (double)(latest_madgwick.first_link * RAD_TO_DEG),
             (double)(latest_madgwick.second_link * RAD_TO_DEG),
             (double)(latest_madgwick.wrist_pitch * RAD_TO_DEG),
             (double)(latest_madgwick.wrist_roll * RAD_TO_DEG),
             (double)(latest_complementary.turntable * RAD_TO_DEG),
             (double)(latest_complementary.first_link * RAD_TO_DEG),
             (double)(latest_complementary.second_link * RAD_TO_DEG),
             (double)(latest_complementary.wrist_pitch * RAD_TO_DEG),
             (double)(latest_complementary.wrist_roll * RAD_TO_DEG));
    }
    k_mutex_unlock(&orientation_lock);
  }
  return 0;
}
