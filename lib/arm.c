#include <Tarzan/lib/arm.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/* wrapper around gpio pin set dt which returns updated position
 * param:
 * motor - dt spec for stepper motor
 * dir - clockwise or anti-clockwise
 * pos - stepper position*/
int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos) {
  if (dir == HIGH_PULSE) {
    gpio_pin_set_dt(&(motor->dir), 1);
    pos += 1; // clockwise
  } else if (dir == LOW_PULSE) {
    gpio_pin_set_dt(&(motor->dir), 0);
    pos -= 1; // anticlockwise
  } else
    return pos;
  switch (pos & 0x03) {
  case 0:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  case 1:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 2:
    gpio_pin_set_dt(&(motor->step), 1);
    break;
  case 3:
    gpio_pin_set_dt(&(motor->step), 0);
    break;
  }
  return pos;
}

void joint_init(struct joint *data) {
  if (data == NULL) {
    return;
  }

  memset(data, 0, sizeof(*data));
  data->mag_scale[0] = 1.0f;
  data->mag_scale[1] = 1.0f;
  data->mag_scale[2] = 1.0f;
  madgwick_init(&data->orientation_filter, MADGWICK_BETA_DEFAULT);
}

/*Calibration for imu
 * param:
 * dev: imu device
 * data: struct joint
 * returns: 0 if calibration succesfull*/
int calibrate_gyro(const struct device *dev, struct joint *data) {
  if (dev == NULL || data == NULL) {
    return 1;
  }

  struct sensor_value gyro[3];
  int rc;

  for (int axis = 0; axis < 3; axis++) {
    data->gyro_offset[axis] = 0.0f;
  }

  for (int sample = 0; sample < 1000; sample++) {
    rc = sensor_sample_fetch(dev);
    if (rc != 0) {
      return 1;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc != 0) {
      return 1;
    }

    for (int axis = 0; axis < 3; axis++) {
      data->gyro_offset[axis] += sensor_value_to_float(&gyro[axis]);
    }
    k_sleep(K_MSEC(1));
  }

  for (int axis = 0; axis < 3; axis++) {
    data->gyro_offset[axis] /= 1000.0f;
  }

  return 0;
}

/* complementary filter to compute pitch & roll
 * param:
 * dev: imu device
 * data: struct joint
 * returns: 0 if succesfull*/
int complementary_filter(const struct device *dev, struct joint *data) {

  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  float dt = (current_time - data->prev_time) / 1000.0;

  if (!sensor_sample_fetch(dev)) {

    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel))
      return 1;
    if (sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro))
      return 1;

    data->prev_time = current_time;
    data->accel[0] = sensor_value_to_float(&accel[0]);
    data->accel[1] = sensor_value_to_float(&accel[1]);
    data->accel[2] = sensor_value_to_float(&accel[2]);
    data->gyro[0] = sensor_value_to_float(&gyro[0]) - data->gyro_offset[0];
    data->gyro[1] = sensor_value_to_float(&gyro[1]) - data->gyro_offset[1];
    data->gyro[2] = sensor_value_to_float(&gyro[2]) - data->gyro_offset[2];

    float pitch_acc =
        (atan2f(-1 * data->accel[0],
                sqrtf(powf(data->accel[1], 2) + powf(data->accel[2], 2))));
    float roll_acc =
        (atan2f(-1 * data->accel[1],
                sqrtf(powf(data->accel[2], 2) + powf(data->accel[0], 2))));
    data->pitch =
        TAU * (data->pitch + (data->gyro[1]) * (dt)) + (1 - TAU) * pitch_acc;
    data->roll =
        TAU * (data->roll + (data->gyro[2]) * (dt)) + (1 - TAU) * roll_acc;

    return 0;
  }
  return 1;
}

/* proportional feedback update for stepper motor
 * param:
 * target_angel - angle of target position to reach
 * current_angel - angel of current position
 * returns: dir to move stepper */
enum StepperDirection update_proportional(float target_angel,
                                          float current_angel) {
  float error = fabsf(target_angel - current_angel);
  if (error <= 0.1f)
    return STOP_PULSE;
  if (target_angel > current_angel)
    return HIGH_PULSE;
  else // target_angel<current_angel
    return LOW_PULSE;
}

/* compute ypr using magnetometer + imu data
 * param:
 * mag_dev - magnetometer device
 * imu_dev - imu device
 * data - pointer to struct joint
 * returns: 0 if successfull*/
int madgwick_filter(const struct device *const mag_dev,
                    const struct device *const imu_dev, struct joint *data) {

  if (mag_dev == NULL || imu_dev == NULL || data == NULL) {
    return 1;
  }

  struct sensor_value mag[3];
  struct sensor_value accel[3];
  struct sensor_value gyro[3];

  uint64_t current_time = k_uptime_get();

  if (data->prev_time == 0U) {
    data->prev_time = current_time;
    return 1;
  }

  float dt = (current_time - data->prev_time) / 1000.0f;
  data->prev_time = current_time;

  if (dt <= 0.0f || dt > 0.5f) {
    return 1;
  }

  if (!sensor_sample_fetch(mag_dev) && !sensor_sample_fetch(imu_dev)) {
    if (sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, mag))
      return 1;
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel))
      return 1;
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro))
      return 1;

    for (int i = 0; i < 3; i++) {
      data->accel[i] = sensor_value_to_float(&accel[i]);
      data->gyro[i] = sensor_value_to_float(&gyro[i]) - data->gyro_offset[i];
      data->mag[i] =
          (sensor_value_to_float(&mag[i]) - data->mag_offset[i]) *
          data->mag_scale[i];
    }

    if (!madgwick_update(
            &data->orientation_filter, data->accel[0], data->accel[1],
            data->accel[2], data->gyro[0], data->gyro[1], data->gyro[2],
            data->mag[0], data->mag[1], data->mag[2], dt)) {
      return 1;
    }

    madgwick_get_euler(&data->orientation_filter.orientation, &data->roll,
                       &data->pitch, &data->yaw);

    return 0;
  }

  return 1;
}

/* calibrate magnetometer
 * param:
 * dev - magnetometer device
 * data - pointer to struct joint
 * returns: 0 if successfull*/
int calibrate_magnetometer(const struct device *dev, struct joint *data) {

  if (dev == NULL || data == NULL) {
    return 1;
  }

  struct sensor_value mag[3];

  float mag_max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  float mag_min[3] = { FLT_MAX,  FLT_MAX,  FLT_MAX};

  int rc;

  const int samples = 4000;

  for (int axis = 0; axis < 3; axis++) {
    data->mag_offset[axis] = 0.0f;
    data->mag_scale[axis] = 1.0f;
  }

  for (int sample = 0; sample < samples; sample++) {

    rc = sensor_sample_fetch(dev);
    if (rc != 0) {
      return 1;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);
    if (rc != 0) {
      return 1;
    }

    for (int axis = 0; axis < 3; axis++) {
      float val = sensor_value_to_float(&mag[axis]);

      if (val > mag_max[axis]) {
        mag_max[axis] = val;
      }

      if (val < mag_min[axis]) {
        mag_min[axis] = val;
      }
    }

    k_sleep(K_MSEC(5));
  }

  float radius[3];

  for (int axis = 0; axis < 3; axis++) {
    data->mag_offset[axis] = 0.5f * (mag_max[axis] + mag_min[axis]);
    radius[axis] = 0.5f * (mag_max[axis] - mag_min[axis]);

    if (radius[axis] <= 1e-6f) {
      return 2;
    }
  }

  float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;

  for (int axis = 0; axis < 3; axis++) {
    data->mag_scale[axis] = avg_radius / radius[axis];
  }

  return 0;
}
