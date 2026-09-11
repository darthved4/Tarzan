#include <Tarzan/lib/arm_pose.h>
#include <Tarzan/lib/pid.h>

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define CONTROL_MS 10U
#define STEP_TICK_US 50U
#define STEP_TICK_HZ (1000000U / STEP_TICK_US)
#define MAX_RATE_PPS 300
#define MAX_ACCEL_PPS2 300
#define MAX_TARGET_DEG 5.0f
#define DEADBAND_DEG 0.5f
#define SENSOR_FAILURE_LIMIT 5U
#define GYRO_CAL_SAMPLES 500U
#define WARMUP_SAMPLES 200U
#define RAD_TO_DEG 57.29577951308232088f
#define DEG_TO_RAD 0.01745329251994329577f
#define PI_F 3.14159265358979323846f

enum joint_id { TT, FL, SL, WP, WR, JOINT_COUNT };
enum arm_mode { ARM_DISABLED, ARM_CLOSED_LOOP, ARM_FAULT };

static const char *const joint_names[JOINT_COUNT] = {"tt", "fl", "sl", "wp",
                                                     "wr"};

struct motor_output {
  struct gpio_dt_spec direction;
  struct gpio_dt_spec step;
  float pulses_per_joint_rev;
  float direction_sign;
};

/* Only FL=3260 is confirmed. Remaining pulse conversions and all direction
 * signs are explicit placeholders to replace after drivetrain confirmation. */
static const struct motor_output motors[JOINT_COUNT] = {
    [TT] = {GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_tt), dir_gpios),
            GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_tt), step_gpios), 3260.0f,
            1.0f},
    [FL] = {GPIO_DT_SPEC_GET(DT_NODELABEL(yako), dir_gpios),
            GPIO_DT_SPEC_GET(DT_NODELABEL(yako), step_gpios), 3260.0f, 1.0f},
    [SL] = {GPIO_DT_SPEC_GET(DT_NODELABEL(rhino), dir_gpios),
            GPIO_DT_SPEC_GET(DT_NODELABEL(rhino), step_gpios), 3260.0f, 1.0f},
    [WP] = {GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_pr1), dir_gpios),
            GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_pr1), step_gpios), 3260.0f,
            1.0f},
    [WR] = {GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_pr2), dir_gpios),
            GPIO_DT_SPEC_GET(DT_NODELABEL(dm5422_pr2), step_gpios), 3260.0f,
            1.0f},
};

struct pulse_generator {
  uint32_t accumulator;
  int direction;
  bool direction_set;
  bool high;
};

struct joint_control {
  struct pid_controller pid;
  struct pid_config gains;
  float target;
};

static struct {
  struct k_mutex lock;
  struct arm_pose_state pose;
  struct joint_control joint[JOINT_COUNT];
  enum arm_mode mode;
  unsigned int sensor_failures;
  bool ready;
} arm;

static struct pulse_generator pulse[JOINT_COUNT];
static atomic_t applied_rate[JOINT_COUNT];

static float measurement(enum joint_id joint) {
  switch (joint) {
  case TT:
    return arm.pose.madgwick.turntable;
  case FL:
    return arm.pose.madgwick.first_link;
  case SL:
    return arm.pose.madgwick.second_link;
  case WP:
    return arm.pose.madgwick.wrist_pitch;
  case WR:
    return arm.pose.madgwick.wrist_roll;
  default:
    return 0.0f;
  }
}

static void step_timer_handler(struct k_timer *timer) {
  ARG_UNUSED(timer);
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    struct pulse_generator *generator = &pulse[joint];
    const int requested = (int)atomic_get(&applied_rate[joint]);
    if (generator->high) {
      gpio_pin_set_dt(&motors[joint].step, 0);
      generator->high = false;
      continue;
    }
    if (requested == 0) {
      generator->accumulator = 0U;
      continue;
    }
    const int direction = requested > 0 ? 1 : -1;
    if (!generator->direction_set || direction != generator->direction) {
      gpio_pin_set_dt(&motors[joint].direction, direction > 0 ? 1 : 0);
      generator->direction = direction;
      generator->direction_set = true;
      generator->accumulator = 0U;
      continue;
    }
    generator->accumulator += (uint32_t)abs(requested);
    if (generator->accumulator >= STEP_TICK_HZ) {
      generator->accumulator -= STEP_TICK_HZ;
      gpio_pin_set_dt(&motors[joint].step, 1);
      generator->high = true;
    }
  }
}

K_TIMER_DEFINE(step_timer, step_timer_handler, NULL);

static void stop_locked(enum arm_mode mode) {
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    atomic_set(&applied_rate[joint], 0);
    pid_reset(&arm.joint[joint].pid);
  }
  arm.mode = mode;
}

static int velocity_to_rate(enum joint_id joint, float velocity) {
  const float rate = velocity * motors[joint].direction_sign *
                     motors[joint].pulses_per_joint_rev / (2.0f * PI_F);
  return isfinite(rate)
             ? CLAMP((int)lroundf(rate), -MAX_RATE_PPS, MAX_RATE_PPS)
             : 0;
}

static void slew_rates(const int desired[JOINT_COUNT]) {
  const int max_change = MAX(1, MAX_ACCEL_PPS2 * (int)CONTROL_MS / 1000);
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    const int current = (int)atomic_get(&applied_rate[joint]);
    atomic_set(&applied_rate[joint],
               current + CLAMP(desired[joint] - current, -max_change,
                               max_change));
  }
}

static int initialize_motors(void) {
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    if (!gpio_is_ready_dt(&motors[joint].direction) ||
        !gpio_is_ready_dt(&motors[joint].step)) {
      return -ENODEV;
    }
    int rc = gpio_pin_configure_dt(&motors[joint].direction,
                                   GPIO_OUTPUT_INACTIVE);
    if (rc == 0) {
      rc = gpio_pin_configure_dt(&motors[joint].step, GPIO_OUTPUT_INACTIVE);
    }
    if (rc != 0) {
      return rc;
    }
    atomic_set(&applied_rate[joint], 0);
  }
  k_timer_start(&step_timer, K_NO_WAIT, K_USEC(STEP_TICK_US));
  return 0;
}

static int initialize_pose(void) {
  const struct arm_pose_config config = {
      .first_link_imu = DEVICE_DT_GET(DT_NODELABEL(first_link_imu)),
      .second_link_imu = DEVICE_DT_GET(DT_NODELABEL(second_link_imu)),
      .end_effector_imu = DEVICE_DT_GET(DT_NODELABEL(end_effector_imu)),
      .turntable_mag = DEVICE_DT_GET(DT_NODELABEL(turntable_mag)),
      .base_reference_mag = DEVICE_DT_GET(DT_NODELABEL(base_reference_mag)),
      .end_effector_mag = DEVICE_DT_GET(DT_NODELABEL(end_effector_mag)),
      .first_link_map = SENSOR_AXIS_MAP_IDENTITY,
      .second_link_map = SENSOR_AXIS_MAP_IDENTITY,
      .end_effector_imu_map = SENSOR_AXIS_MAP_IDENTITY,
      .end_effector_mag_map = SENSOR_AXIS_MAP_IDENTITY,
      .madgwick_beta = MADGWICK_BETA_DEFAULT,
      .complementary_alpha = 0.98f,
  };
  int rc = arm_pose_init(&arm.pose, &config);
  if (rc != 0) {
    return rc;
  }
  printk("Keep all links stationary during gyro calibration\n");
  rc = arm_pose_calibrate_gyros(&arm.pose, GYRO_CAL_SAMPLES, 2U);
  for (unsigned int i = 0U; rc == 0 && i < WARMUP_SAMPLES; i++) {
    rc = arm_pose_update(&arm.pose, (float)CONTROL_MS / 1000.0f);
    k_sleep(K_MSEC(CONTROL_MS));
  }
  return rc;
}

static int joint_from_name(const char *name) {
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    if (strcmp(name, joint_names[joint]) == 0) {
      return joint;
    }
  }
  return -1;
}

static bool parse_float(const char *text, float *value) {
  char *end = NULL;
  errno = 0;
  *value = strtof(text, &end);
  return errno == 0 && end != text && *end == '\0' && isfinite(*value);
}

static int arm_command(const struct shell *shell, size_t argc, char **argv) {
  if (argc < 2U) {
    shell_error(shell, "Usage: arm status|stop|zero|target|target-all|gains");
    return -EINVAL;
  }
  if (strcmp(argv[1], "status") == 0) {
    k_mutex_lock(&arm.lock, K_FOREVER);
    shell_print(shell, "mode=%d ready=%d failures=%u", arm.mode, arm.ready,
                arm.sensor_failures);
    for (int joint = 0; joint < JOINT_COUNT; joint++) {
      shell_print(shell, "%s target=%.2f angle=%.2f rate=%d", joint_names[joint],
                  (double)(arm.joint[joint].target * RAD_TO_DEG),
                  (double)(measurement(joint) * RAD_TO_DEG),
                  (int)atomic_get(&applied_rate[joint]));
    }
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  if (strcmp(argv[1], "stop") == 0) {
    k_mutex_lock(&arm.lock, K_FOREVER);
    stop_locked(ARM_DISABLED);
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  if (strcmp(argv[1], "zero") == 0) {
    k_mutex_lock(&arm.lock, K_FOREVER);
    if (arm.mode != ARM_DISABLED) {
      k_mutex_unlock(&arm.lock);
      return -EBUSY;
    }
    arm_pose_zero(&arm.pose);
    for (int joint = 0; joint < JOINT_COUNT; joint++) {
      arm.joint[joint].target = 0.0f;
      pid_reset(&arm.joint[joint].pid);
    }
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  if (strcmp(argv[1], "target") == 0) {
    float degrees;
    const int joint = argc == 4U ? joint_from_name(argv[2]) : -1;
    if (joint < 0 || !parse_float(argv[3], &degrees) ||
        fabsf(degrees) > MAX_TARGET_DEG) {
      shell_error(shell, "Usage: arm target <tt|fl|sl|wp|wr> <-5..5 deg>");
      return -EINVAL;
    }
    k_mutex_lock(&arm.lock, K_FOREVER);
    if (!arm.ready || arm.mode == ARM_FAULT) {
      k_mutex_unlock(&arm.lock);
      return -EIO;
    }
    for (int other = 0; other < JOINT_COUNT; other++) {
      arm.joint[other].target = measurement(other);
      pid_reset(&arm.joint[other].pid);
    }
    arm.joint[joint].target = degrees * DEG_TO_RAD;
    arm.mode = ARM_CLOSED_LOOP;
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  if (strcmp(argv[1], "target-all") == 0) {
    float degrees[JOINT_COUNT];
    if (argc != 7U) {
      shell_error(shell, "Usage: arm target-all <tt> <fl> <sl> <wp> <wr>");
      return -EINVAL;
    }
    for (int joint = 0; joint < JOINT_COUNT; joint++) {
      if (!parse_float(argv[joint + 2], &degrees[joint]) ||
          fabsf(degrees[joint]) > MAX_TARGET_DEG) {
        shell_error(shell, "Every target must be within +/-5 degrees");
        return -EINVAL;
      }
    }
    k_mutex_lock(&arm.lock, K_FOREVER);
    if (!arm.ready || arm.mode == ARM_FAULT) {
      k_mutex_unlock(&arm.lock);
      return -EIO;
    }
    for (int joint = 0; joint < JOINT_COUNT; joint++) {
      arm.joint[joint].target = degrees[joint] * DEG_TO_RAD;
      pid_reset(&arm.joint[joint].pid);
    }
    arm.mode = ARM_CLOSED_LOOP;
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  if (strcmp(argv[1], "gains") == 0) {
    float kp;
    float ki;
    float kd;
    const int joint = argc == 6U ? joint_from_name(argv[2]) : -1;
    if (joint < 0 || !parse_float(argv[3], &kp) ||
        !parse_float(argv[4], &ki) || !parse_float(argv[5], &kd) || kp < 0.0f ||
        ki < 0.0f || kd < 0.0f) {
      shell_error(shell, "Usage: arm gains <joint> <kp> <ki> <kd>");
      return -EINVAL;
    }
    k_mutex_lock(&arm.lock, K_FOREVER);
    if (arm.mode != ARM_DISABLED) {
      k_mutex_unlock(&arm.lock);
      return -EBUSY;
    }
    arm.joint[joint].gains.kp = kp;
    arm.joint[joint].gains.ki = ki;
    arm.joint[joint].gains.kd = kd;
    pid_init(&arm.joint[joint].pid, &arm.joint[joint].gains);
    k_mutex_unlock(&arm.lock);
    return 0;
  }
  return -EINVAL;
}

SHELL_CMD_REGISTER(arm, NULL, "Whole-arm closed-loop PID demo", arm_command);

int main(void) {
  printk("\n=== Tarzan whole-arm closed-loop PID ===\n");
  k_mutex_init(&arm.lock);
  arm.mode = ARM_DISABLED;
  const struct pid_config defaults = {
      .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
      .output_min = -0.20f, .output_max = 0.20f,
      .integral_min = -0.04f, .integral_max = 0.04f,
      .derivative_filter_tau = 0.05f};
  for (int joint = 0; joint < JOINT_COUNT; joint++) {
    arm.joint[joint].gains = defaults;
    pid_init(&arm.joint[joint].pid, &arm.joint[joint].gains);
  }

  if (initialize_motors() != 0 || initialize_pose() != 0) {
    stop_locked(ARM_FAULT);
    printk("Initialization failed; every motor is disabled\n");
  } else {
    arm.ready = true;
    printk("Ready in disabled mode. Use 'arm status' first.\n");
  }

  int64_t previous_ms = k_uptime_get();
  unsigned int telemetry = 0U;
  while (true) {
    k_sleep(K_MSEC(CONTROL_MS));
    const int64_t now_ms = k_uptime_get();
    const float dt = (float)(now_ms - previous_ms) / 1000.0f;
    previous_ms = now_ms;

    int desired[JOINT_COUNT] = {0};
    k_mutex_lock(&arm.lock, K_FOREVER);
    const int sensor_rc = arm.ready ? arm_pose_update(&arm.pose, dt) : -EIO;
    if (sensor_rc != 0) {
      if (++arm.sensor_failures >= SENSOR_FAILURE_LIMIT) {
        stop_locked(ARM_FAULT);
      }
    } else {
      arm.sensor_failures = 0U;
    }
    if (arm.mode == ARM_CLOSED_LOOP && sensor_rc == 0) {
      float joint_velocity[JOINT_COUNT] = {0};
      for (int joint = 0; joint < JOINT_COUNT; joint++) {
        const float angle = measurement(joint);
        if (fabsf((arm.joint[joint].target - angle) * RAD_TO_DEG) >
            DEADBAND_DEG) {
          joint_velocity[joint] = pid_update(
              &arm.joint[joint].pid, arm.joint[joint].target, angle, dt);
        } else {
          pid_reset(&arm.joint[joint].pid);
        }
      }

      desired[TT] = velocity_to_rate(TT, joint_velocity[TT]);
      desired[FL] = velocity_to_rate(FL, joint_velocity[FL]);
      desired[SL] = velocity_to_rate(SL, joint_velocity[SL]);
      /* TODO(mechanical): replace identity wrist mix with the confirmed PR1/
       * PR2 differential matrix. */
      desired[WP] = velocity_to_rate(WP, joint_velocity[WP]);
      desired[WR] = velocity_to_rate(WR, joint_velocity[WR]);
    }
    const enum arm_mode mode = arm.mode;
    const struct arm_joint_angles angles = arm.pose.madgwick;
    k_mutex_unlock(&arm.lock);
    slew_rates(desired);

    if (++telemetry >= 10U) {
      telemetry = 0U;
      printk("ARM,%lld,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", now_ms, mode,
             (double)(angles.turntable * RAD_TO_DEG),
             (double)(angles.first_link * RAD_TO_DEG),
             (double)(angles.second_link * RAD_TO_DEG),
             (double)(angles.wrist_pitch * RAD_TO_DEG),
             (double)(angles.wrist_roll * RAD_TO_DEG));
    }
  }
  return 0;
}
