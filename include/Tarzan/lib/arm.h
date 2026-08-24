#ifndef ARM_H
#define ARM_H

#include <Tarzan/lib/madgwick.h>

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define M_PI 3.14159265358979323846f
#define TAU 0.90f

struct stepper_motor {
  const struct gpio_dt_spec dir;
  const struct gpio_dt_spec step;
  const int channel;
};

enum StepperDirection {
  LOW_PULSE = 0,
  HIGH_PULSE = 1,
  STOP_PULSE = 2,
};

/* store imu related data */
struct joint {
  float accel[3];
  float gyro[3];
  float mag[3];
  float pitch;
  float roll;
  float yaw;
  float gyro_offset[3];
  float mag_offset[3];
  float mag_scale[3];
  struct madgwick_state orientation_filter;
  uint64_t prev_time;
};

/* Initialize sensor calibration defaults and independent orientation state. */
void joint_init(struct joint *data);

int Stepper_motor_write(const struct stepper_motor *motor, int dir, int pos);

int complementary_filter(const struct device *dev, struct joint *data);

int calibrate_gyro(const struct device *dev, struct joint *data);

int madgwick_filter(const struct device *const mag_dev,
                    const struct device *const imu_dev, struct joint *data);

int calibrate_magnetometer(const struct device *dev, struct joint *data);

enum StepperDirection update_proportional(float target_angle,
                                          float current_angel);
#endif
