#include <Tarzan/lib/pid.h>

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static int failures;

#define EXPECT_TRUE(condition)                                                \
  do {                                                                        \
    if (!(condition)) {                                                       \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);           \
      failures++;                                                             \
    }                                                                         \
  } while (false)

#define EXPECT_NEAR(actual_expression, expected_expression, tolerance)         \
  do {                                                                        \
    const float actual_value = (actual_expression);                           \
    const float expected_value = (expected_expression);                       \
    if (!isfinite(actual_value) ||                                            \
        fabsf(actual_value - expected_value) > (tolerance)) {                 \
      printf("FAIL %s:%d: %s = %.9g, expected %.9g +/- %.9g\n",             \
             __FILE__, __LINE__, #actual_expression, (double)actual_value,    \
             (double)expected_value, (double)(tolerance));                    \
      failures++;                                                             \
    }                                                                         \
  } while (false)

static struct pid_config base_config(void) {
  return (struct pid_config){
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .output_min = -1000.0f,
      .output_max = 1000.0f,
      .integral_min = -1000.0f,
      .integral_max = 1000.0f,
      .derivative_filter_tau = 0.0f,
  };
}

static void expect_dynamic_state_equal(const struct pid_controller *actual,
                                       const struct pid_controller *expected) {
  EXPECT_TRUE(actual->previous_measurement == expected->previous_measurement);
  EXPECT_TRUE(actual->filtered_measurement_derivative ==
              expected->filtered_measurement_derivative);
  EXPECT_TRUE(actual->error == expected->error);
  EXPECT_TRUE(actual->p_term == expected->p_term);
  EXPECT_TRUE(actual->i_term == expected->i_term);
  EXPECT_TRUE(actual->d_term == expected->d_term);
  EXPECT_TRUE(actual->output == expected->output);
  EXPECT_TRUE(actual->has_previous_measurement ==
              expected->has_previous_measurement);
  EXPECT_TRUE(actual->config_valid == expected->config_valid);
}

static void test_zero_error(void) {
  struct pid_config config = base_config();
  config.kp = 2.0f;
  config.ki = 1.0f;
  config.kd = 0.5f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  EXPECT_NEAR(pid_update(&pid, 1.0f, 1.0f, 0.01f), 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid.p_term, 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid.i_term, 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid.d_term, 0.0f, 1.0e-6f);
}

static void test_positive_and_negative_error(void) {
  struct pid_config config = base_config();
  config.kp = 2.0f;

  struct pid_controller positive;
  struct pid_controller negative;
  pid_init(&positive, &config);
  pid_init(&negative, &config);

  EXPECT_NEAR(pid_update(&positive, 1.0f, 0.25f, 0.1f), 1.5f,
              1.0e-6f);
  EXPECT_NEAR(pid_update(&negative, 0.25f, 1.0f, 0.1f), -1.5f,
              1.0e-6f);
}

static void test_output_saturation(void) {
  struct pid_config config = base_config();
  config.kp = 10.0f;
  config.output_min = -1.0f;
  config.output_max = 1.0f;

  struct pid_controller positive;
  struct pid_controller negative;
  pid_init(&positive, &config);
  pid_init(&negative, &config);

  EXPECT_NEAR(pid_update(&positive, 1.0f, 0.0f, 0.1f), 1.0f,
              1.0e-6f);
  EXPECT_NEAR(pid_update(&negative, -1.0f, 0.0f, 0.1f), -1.0f,
              1.0e-6f);
}

static void test_integral_accumulation(void) {
  struct pid_config config = base_config();
  config.ki = 2.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  for (int i = 0; i < 5; i++) {
    pid_update(&pid, 1.0f, 0.0f, 0.1f);
  }

  EXPECT_NEAR(pid.i_term, 1.0f, 1.0e-6f);
  EXPECT_NEAR(pid.output, 1.0f, 1.0e-6f);
}

static void test_integral_clamping(void) {
  struct pid_config config = base_config();
  config.ki = 10.0f;
  config.integral_min = -0.5f;
  config.integral_max = 0.5f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  for (int i = 0; i < 10; i++) {
    pid_update(&pid, 1.0f, 0.0f, 0.1f);
  }

  EXPECT_NEAR(pid.i_term, 0.5f, 1.0e-6f);
  EXPECT_NEAR(pid.output, 0.5f, 1.0e-6f);
}

static void test_integral_anti_windup(void) {
  struct pid_config config = base_config();
  config.ki = 1.0f;
  config.output_min = -1.0f;
  config.output_max = 1.0f;
  config.integral_min = -10.0f;
  config.integral_max = 10.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.0f, 1.0f), 1.0f, 1.0e-6f);
  EXPECT_NEAR(pid.i_term, 1.0f, 1.0e-6f);

  for (int i = 0; i < 10; i++) {
    pid_update(&pid, 1.0f, 0.0f, 1.0f);
  }
  EXPECT_NEAR(pid.i_term, 1.0f, 1.0e-6f);

  EXPECT_NEAR(pid_update(&pid, -1.0f, 0.0f, 0.5f), 0.5f, 1.0e-6f);
  EXPECT_NEAR(pid.i_term, 0.5f, 1.0e-6f);
}

static void test_derivative_response(void) {
  struct pid_config config = base_config();
  config.kd = 2.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  EXPECT_NEAR(pid_update(&pid, 0.0f, 0.0f, 0.25f), 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid_update(&pid, 0.0f, 0.5f, 0.25f), -4.0f,
              1.0e-6f);
  EXPECT_NEAR(pid.filtered_measurement_derivative, 2.0f, 1.0e-6f);
}

static void test_derivative_filtering(void) {
  struct pid_config config = base_config();
  config.kd = 1.0f;
  config.derivative_filter_tau = 0.5f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  pid_update(&pid, 0.0f, 0.0f, 0.5f);
  EXPECT_NEAR(pid_update(&pid, 0.0f, 1.0f, 0.5f), -1.0f,
              1.0e-6f);
  EXPECT_NEAR(pid.filtered_measurement_derivative, 1.0f, 1.0e-6f);

  EXPECT_NEAR(pid_update(&pid, 0.0f, 1.0f, 0.5f), -0.5f,
              1.0e-6f);
  EXPECT_NEAR(pid.filtered_measurement_derivative, 0.5f, 1.0e-6f);
}

static void test_setpoint_change_has_no_derivative_kick(void) {
  struct pid_config config = base_config();
  config.kp = 2.0f;
  config.kd = 3.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  pid_update(&pid, 0.0f, 0.0f, 0.1f);
  EXPECT_NEAR(pid_update(&pid, 10.0f, 0.0f, 0.1f), 20.0f,
              1.0e-6f);
  EXPECT_NEAR(pid.d_term, 0.0f, 1.0e-6f);
}

static void test_reset(void) {
  struct pid_config config = base_config();
  config.kp = 1.0f;
  config.ki = 1.0f;
  config.kd = 1.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);
  pid_update(&pid, 2.0f, 1.0f, 0.1f);
  pid_update(&pid, 2.0f, 1.5f, 0.1f);

  pid_reset(&pid);

  EXPECT_TRUE(pid.config_valid);
  EXPECT_NEAR(pid.config.kp, 1.0f, 1.0e-6f);
  EXPECT_TRUE(!pid.has_previous_measurement);
  EXPECT_NEAR(pid.i_term, 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid.filtered_measurement_derivative, 0.0f, 1.0e-6f);
  EXPECT_NEAR(pid.output, 0.0f, 1.0e-6f);

  EXPECT_NEAR(pid_update(&pid, 2.0f, 1.5f, 0.1f), 0.55f,
              1.0e-6f);
  EXPECT_NEAR(pid.d_term, 0.0f, 1.0e-6f);
}

static void test_invalid_dt_preserves_state(void) {
  struct pid_config config = base_config();
  config.kp = 1.0f;
  config.ki = 1.0f;
  config.kd = 1.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);
  pid_update(&pid, 1.0f, 0.25f, 0.1f);
  const struct pid_controller expected = pid;

  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.25f, 0.0f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.25f, -0.1f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.25f, NAN), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.25f, INFINITY), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
}

static void test_non_finite_inputs_preserve_state(void) {
  struct pid_config config = base_config();
  config.kp = 1.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);
  pid_update(&pid, 1.0f, 0.25f, 0.1f);
  const struct pid_controller expected = pid;

  EXPECT_NEAR(pid_update(&pid, NAN, 0.0f, 0.1f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, INFINITY, 0.0f, 0.1f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, 0.0f, NAN, 0.1f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, 0.0f, -INFINITY, 0.1f), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
  EXPECT_NEAR(pid_update(&pid, FLT_MAX, -FLT_MAX, 0.1f), 0.0f,
              0.0f);
  expect_dynamic_state_equal(&pid, &expected);
}

static void test_non_finite_intermediate_preserves_state(void) {
  struct pid_config config = base_config();
  config.kd = 1.0f;
  config.derivative_filter_tau = FLT_MAX;

  struct pid_controller pid;
  pid_init(&pid, &config);
  pid_update(&pid, 0.0f, 0.0f, 0.1f);
  const struct pid_controller expected = pid;

  EXPECT_NEAR(pid_update(&pid, 0.0f, 1.0f, FLT_MAX), 0.0f, 0.0f);
  expect_dynamic_state_equal(&pid, &expected);
}

static void test_invalid_configuration(void) {
  struct pid_config config = base_config();
  config.kp = -1.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  EXPECT_TRUE(!pid.config_valid);
  EXPECT_NEAR(pid_update(&pid, 1.0f, 0.0f, 0.1f), 0.0f, 0.0f);

  config = base_config();
  config.output_min = 1.0f;
  config.output_max = 2.0f;
  pid_init(&pid, &config);
  EXPECT_TRUE(!pid.config_valid);

  config = base_config();
  config.derivative_filter_tau = NAN;
  pid_init(&pid, &config);
  EXPECT_TRUE(!pid.config_valid);

  pid_init(NULL, NULL);
  pid_reset(NULL);
  EXPECT_NEAR(pid_update(NULL, 0.0f, 0.0f, 0.1f), 0.0f, 0.0f);
}

static void test_deterministic_state_updates(void) {
  struct pid_config config = base_config();
  config.kp = 1.5f;
  config.ki = 0.25f;
  config.kd = 0.1f;
  config.derivative_filter_tau = 0.05f;

  const float setpoints[] = {0.0f, 1.0f, 1.0f, -0.5f, 0.25f};
  const float measurements[] = {0.0f, 0.1f, 0.4f, 0.2f, -0.1f};
  const float times[] = {0.01f, 0.02f, 0.01f, 0.03f, 0.02f};

  struct pid_controller first;
  struct pid_controller second;
  pid_init(&first, &config);
  pid_init(&second, &config);

  for (unsigned int i = 0;
       i < sizeof(setpoints) / sizeof(setpoints[0]); i++) {
    const float first_output =
        pid_update(&first, setpoints[i], measurements[i], times[i]);
    const float second_output =
        pid_update(&second, setpoints[i], measurements[i], times[i]);
    EXPECT_TRUE(first_output == second_output);
    expect_dynamic_state_equal(&first, &second);
  }
}

static void test_simulated_plant_convergence(void) {
  struct pid_config config = base_config();
  config.kp = 3.0f;
  config.output_min = -1.0f;
  config.output_max = 1.0f;
  config.integral_min = -1.0f;
  config.integral_max = 1.0f;

  struct pid_controller pid;
  pid_init(&pid, &config);

  const float target = 1.0f;
  const float dt = 0.01f;
  float position = 0.0f;

  for (int i = 0; i < 500; i++) {
    const float velocity = pid_update(&pid, target, position, dt);
    EXPECT_TRUE(isfinite(velocity));
    EXPECT_TRUE(velocity >= config.output_min);
    EXPECT_TRUE(velocity <= config.output_max);
    position += velocity * dt;
  }

  EXPECT_NEAR(position, target, 1.0e-3f);
}

int main(void) {
  test_zero_error();
  test_positive_and_negative_error();
  test_output_saturation();
  test_integral_accumulation();
  test_integral_clamping();
  test_integral_anti_windup();
  test_derivative_response();
  test_derivative_filtering();
  test_setpoint_change_has_no_derivative_kick();
  test_reset();
  test_invalid_dt_preserves_state();
  test_non_finite_inputs_preserve_state();
  test_non_finite_intermediate_preserves_state();
  test_invalid_configuration();
  test_deterministic_state_updates();
  test_simulated_plant_convergence();

  if (failures != 0) {
    printf("PID tests failed: %d\n", failures);
    return 1;
  }

  printf("All PID tests passed.\n");
  return 0;
}
