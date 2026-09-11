#include <Tarzan/lib/link_orientation.h>
#include <Tarzan/lib/pid.h>

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/* First Link feedback comes from its MPU6050 orientation. STEP pulses are
 * actuation only and are never position feedback. */
#define YAKO_NODE DT_NODELABEL(yako)
#define FL_IMU_NODE DT_NODELABEL(first_link_imu)

#define CONTROL_MS 10U
#define STEP_TICK_US 50U
#define STEP_TICK_HZ (1000000U / STEP_TICK_US)
#define MAX_RATE_PPS 500
#define MAX_ACCEL_PPS2 500
#define MAX_TARGET_DEG 5.0f
#define DEADBAND_DEG 0.5f
#define SENSOR_FAILURE_LIMIT 5U
#define GYRO_CAL_SAMPLES 500U
#define WARMUP_SAMPLES 200U
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232088f
#define PI_F 3.14159265358979323846f

/* 200 steps/rev * 2 pulses/step * provisional 8.15:1 gearbox. This is only
 * output-unit conversion, not an angle estimator. */
#define PULSES_PER_JOINT_REV 3260.0f

/* TODO(mechanical): replace identity maps and confirm the +Y hinge axis. */
static const struct sensor_axis_map identity_map = {
    .source_axis = {0U, 1U, 2U}, .sign = {1, 1, 1}};

static const struct gpio_dt_spec yako_dir =
    GPIO_DT_SPEC_GET(YAKO_NODE, dir_gpios);
static const struct gpio_dt_spec yako_step =
    GPIO_DT_SPEC_GET(YAKO_NODE, step_gpios);

enum mode { DISABLED, OPEN_LOOP, CLOSED_LOOP, FAULT };

static struct {
  struct k_mutex lock;
  struct link_orientation first_link;
  struct pid_controller pid;
  struct pid_config gains;
  enum mode mode;
  float target;
  float angle;
  float comp_angle;
  float zero;
  float comp_zero;
  int jog_rate;
  unsigned int failures;
  bool ready;
} demo;

static atomic_t applied_rate_pps;

static const char *mode_name(enum mode mode) {
  static const char *const names[] = {"disabled", "open-loop", "closed-loop",
                                      "fault"};
  return mode <= FAULT ? names[mode] : "unknown";
}

static float wrap(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static int velocity_to_rate(float velocity) {
  const float rate = velocity * PULSES_PER_JOINT_REV / (2.0f * PI_F);
  return isfinite(rate)
             ? CLAMP((int)lroundf(rate), -MAX_RATE_PPS, MAX_RATE_PPS)
             : 0;
}

static void step_timer_handler(struct k_timer *timer) {
  ARG_UNUSED(timer);
  static uint32_t accumulator;
  static bool high;
  static int direction;
  static bool direction_set;
  const int requested = (int)atomic_get(&applied_rate_pps);

  if (high) {
    gpio_pin_set_dt(&yako_step, 0);
    high = false;
    return;
  }
  if (requested == 0) {
    accumulator = 0U;
    return;
  }
  const int requested_direction = requested > 0 ? 1 : -1;
  if (!direction_set || requested_direction != direction) {
    gpio_pin_set_dt(&yako_dir, requested_direction > 0 ? 1 : 0);
    direction = requested_direction;
    direction_set = true;
    accumulator = 0U;
    return;
  }
  accumulator += (uint32_t)abs(requested);
  if (accumulator >= STEP_TICK_HZ) {
    accumulator -= STEP_TICK_HZ;
    gpio_pin_set_dt(&yako_step, 1);
    high = true;
  }
}

K_TIMER_DEFINE(step_timer, step_timer_handler, NULL);

static void stop_locked(enum mode next_mode) {
  demo.jog_rate = 0;
  atomic_set(&applied_rate_pps, 0);
  demo.mode = next_mode;
  pid_reset(&demo.pid);
}

static void slew_rate(int desired) {
  const int current = (int)atomic_get(&applied_rate_pps);
  const int maximum_change = MAX(1, MAX_ACCEL_PPS2 * (int)CONTROL_MS / 1000);
  atomic_set(&applied_rate_pps,
             current + CLAMP(desired - current, -maximum_change,
                             maximum_change));
}

static int sample_angle(float dt, float *madgwick, float *complementary) {
  const int rc = link_orientation_update(&demo.first_link, dt);
  if (rc != 0) {
    return rc;
  }

  float roll;
  float yaw;
  if (!orientation_get_euler_rad(&demo.first_link.estimator, &roll, madgwick,
                                 &yaw)) {
    return -EIO;
  }
  *complementary = demo.first_link.estimator.complementary.pitch_rad;
  return 0;
}

static int initialize_orientation(void) {
  int rc = link_orientation_init(&demo.first_link, DEVICE_DT_GET(FL_IMU_NODE),
                                 NULL, false, &identity_map, &identity_map,
                                 MADGWICK_BETA_DEFAULT, 0.98f);
  if (rc != 0) {
    return rc;
  }

  printk("Keep the First Link stationary\n");
  rc = link_orientation_calibrate_gyro(&demo.first_link, GYRO_CAL_SAMPLES,
                                       2U);

  float angle = 0.0f;
  float comp = 0.0f;
  for (unsigned int i = 0U; rc == 0 && i < WARMUP_SAMPLES; i++) {
    rc = sample_angle((float)CONTROL_MS / 1000.0f, &angle, &comp);
    k_sleep(K_MSEC(CONTROL_MS));
  }
  if (rc == 0) {
    demo.zero = angle;
    demo.comp_zero = comp;
    demo.ready = true;
  }
  return rc;
}

static int initialize_motor(void) {
  if (!gpio_is_ready_dt(&yako_dir) || !gpio_is_ready_dt(&yako_step)) {
    return -ENODEV;
  }
  int rc = gpio_pin_configure_dt(&yako_dir, GPIO_OUTPUT_INACTIVE);
  if (rc == 0) {
    rc = gpio_pin_configure_dt(&yako_step, GPIO_OUTPUT_INACTIVE);
  }
  if (rc == 0) {
    atomic_set(&applied_rate_pps, 0);
    k_timer_start(&step_timer, K_NO_WAIT, K_USEC(STEP_TICK_US));
  }
  return rc;
}

static bool parse_float(const char *text, float *value) {
  char *end = NULL;
  errno = 0;
  *value = strtof(text, &end);
  return errno == 0 && end != text && *end == '\0' && isfinite(*value);
}

static int yako_command(const struct shell *shell, size_t argc, char **argv) {
  if (argc < 2U) {
    shell_error(shell, "Usage: yako status|jog|stop|zero|gains|target");
    return -EINVAL;
  }

  if (strcmp(argv[1], "status") == 0) {
    k_mutex_lock(&demo.lock, K_FOREVER);
    shell_print(shell,
                "mode=%s target=%.2fdeg madgwick=%.2fdeg complementary=%.2fdeg rate=%dpps failures=%u",
                mode_name(demo.mode), (double)(demo.target * RAD_TO_DEG),
                (double)(demo.angle * RAD_TO_DEG),
                (double)(demo.comp_angle * RAD_TO_DEG),
                (int)atomic_get(&applied_rate_pps), demo.failures);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  if (strcmp(argv[1], "stop") == 0) {
    k_mutex_lock(&demo.lock, K_FOREVER);
    stop_locked(DISABLED);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  if (strcmp(argv[1], "jog") == 0) {
    if (argc != 3U) {
      return -EINVAL;
    }
    char *end = NULL;
    errno = 0;
    const long rate = strtol(argv[2], &end, 10);
    if (errno != 0 || end == argv[2] || *end != '\0' ||
        rate < -MAX_RATE_PPS || rate > MAX_RATE_PPS) {
      shell_error(shell, "Jog range is %d to %d pps", -MAX_RATE_PPS,
                  MAX_RATE_PPS);
      return -EINVAL;
    }
    k_mutex_lock(&demo.lock, K_FOREVER);
    demo.jog_rate = (int)rate;
    demo.mode = rate == 0 ? DISABLED : OPEN_LOOP;
    pid_reset(&demo.pid);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  if (strcmp(argv[1], "zero") == 0) {
    k_mutex_lock(&demo.lock, K_FOREVER);
    if (demo.mode != DISABLED) {
      k_mutex_unlock(&demo.lock);
      shell_error(shell, "Stop Yako before zeroing");
      return -EBUSY;
    }
    demo.zero = wrap(demo.zero + demo.angle);
    demo.comp_zero = wrap(demo.comp_zero + demo.comp_angle);
    demo.angle = 0.0f;
    demo.comp_angle = 0.0f;
    demo.target = 0.0f;
    pid_reset(&demo.pid);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  if (strcmp(argv[1], "gains") == 0) {
    float kp;
    float ki;
    float kd;
    if (argc != 5U || !parse_float(argv[2], &kp) ||
        !parse_float(argv[3], &ki) || !parse_float(argv[4], &kd) || kp < 0.0f ||
        ki < 0.0f || kd < 0.0f) {
      shell_error(shell, "Usage: yako gains <nonnegative kp> <ki> <kd>");
      return -EINVAL;
    }
    k_mutex_lock(&demo.lock, K_FOREVER);
    if (demo.mode != DISABLED) {
      k_mutex_unlock(&demo.lock);
      return -EBUSY;
    }
    demo.gains.kp = kp;
    demo.gains.ki = ki;
    demo.gains.kd = kd;
    pid_init(&demo.pid, &demo.gains);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  if (strcmp(argv[1], "target") == 0) {
    float degrees;
    if (argc != 3U || !parse_float(argv[2], &degrees) ||
        fabsf(degrees) > MAX_TARGET_DEG) {
      shell_error(shell, "Target range is %.1f to %.1f degrees",
                  (double)-MAX_TARGET_DEG, (double)MAX_TARGET_DEG);
      return -EINVAL;
    }
    k_mutex_lock(&demo.lock, K_FOREVER);
    if (!demo.ready || demo.mode == FAULT) {
      k_mutex_unlock(&demo.lock);
      return -EIO;
    }
    demo.target = degrees * DEG_TO_RAD;
    demo.mode = CLOSED_LOOP;
    pid_reset(&demo.pid);
    k_mutex_unlock(&demo.lock);
    return 0;
  }
  return -EINVAL;
}

SHELL_CMD_REGISTER(yako, NULL, "First Link IMU-feedback PID", yako_command);

int main(void) {
  printk("\n=== Tarzan First Link / Yako IMU PID ===\n");
  k_mutex_init(&demo.lock);
  demo.mode = DISABLED;
  demo.gains = (struct pid_config){
      .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
      .output_min = -0.25f, .output_max = 0.25f,
      .integral_min = -0.05f, .integral_max = 0.05f,
      .derivative_filter_tau = 0.05f};
  pid_init(&demo.pid, &demo.gains);

  if (initialize_motor() != 0 || initialize_orientation() != 0) {
    stop_locked(FAULT);
    printk("Initialization failed; motor disabled\n");
  } else {
    printk("Ready. Begin with 'yako status'; closed-loop limit is +/-5 deg.\n");
  }

  int64_t previous_ms = k_uptime_get();
  unsigned int telemetry = 0U;
  while (true) {
    k_sleep(K_MSEC(CONTROL_MS));
    const int64_t now_ms = k_uptime_get();
    const float dt = (float)(now_ms - previous_ms) / 1000.0f;
    previous_ms = now_ms;

    float raw = 0.0f;
    float raw_comp = 0.0f;
    const int sensor_rc = demo.ready ? sample_angle(dt, &raw, &raw_comp) : -EIO;
    k_mutex_lock(&demo.lock, K_FOREVER);
    if (sensor_rc != 0) {
      if (++demo.failures >= SENSOR_FAILURE_LIMIT) {
        stop_locked(FAULT);
      }
    } else {
      demo.failures = 0U;
      demo.angle = wrap(raw - demo.zero);
      demo.comp_angle = wrap(raw_comp - demo.comp_zero);
    }

    int desired = 0;
    if (demo.mode == OPEN_LOOP) {
      desired = demo.jog_rate;
    } else if (demo.mode == CLOSED_LOOP && sensor_rc == 0) {
      if (fabsf((demo.target - demo.angle) * RAD_TO_DEG) > DEADBAND_DEG) {
        desired = velocity_to_rate(
            pid_update(&demo.pid, demo.target, demo.angle, dt));
      } else {
        pid_reset(&demo.pid);
      }
    }
    const enum mode mode = demo.mode;
    const float target = demo.target * RAD_TO_DEG;
    const float angle = demo.angle * RAD_TO_DEG;
    const float comp = demo.comp_angle * RAD_TO_DEG;
    const float output = demo.pid.output;
    k_mutex_unlock(&demo.lock);
    slew_rate(desired);

    if (++telemetry >= 10U) {
      telemetry = 0U;
      printk("YAKO,%lld,%s,%.3f,%.3f,%.3f,%.3f,%d\n", now_ms,
             mode_name(mode), (double)target, (double)angle, (double)comp,
             (double)output, (int)atomic_get(&applied_rate_pps));
    }
  }
  return 0;
}
