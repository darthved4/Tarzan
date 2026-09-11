#include <Tarzan/lib/orientation.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define PI_F 3.14159265358979323846f

static int failures;

#define EXPECT_TRUE(condition)                                                 \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);            \
      failures++;                                                              \
    }                                                                          \
  } while (false)

#define EXPECT_NEAR(actual_expression, expected_expression, tolerance)          \
  do {                                                                         \
    const float actual_value = (actual_expression);                            \
    const float expected_value = (expected_expression);                        \
    if (!isfinite(actual_value) ||                                             \
        fabsf(actual_value - expected_value) > (tolerance)) {                  \
      printf("FAIL %s:%d: %s = %.7f, expected %.7f +/- %.7f\n",              \
             __FILE__, __LINE__, #actual_expression, (double)actual_value,     \
             (double)expected_value, (double)(tolerance));                     \
      failures++;                                                              \
    }                                                                          \
  } while (false)

static struct quaternion z_rotation(float angle_rad) {
  return (struct quaternion){
      .w = cosf(angle_rad * 0.5f),
      .x = 0.0f,
      .y = 0.0f,
      .z = sinf(angle_rad * 0.5f),
  };
}

static struct quaternion axis_rotation(float x, float y, float z,
                                       float angle_rad) {
  const float half_sine = sinf(angle_rad * 0.5f);
  return (struct quaternion){
      .w = cosf(angle_rad * 0.5f),
      .x = x * half_sine,
      .y = y * half_sine,
      .z = z * half_sine,
  };
}

static void test_stationary_filters(void) {
  struct orientation_estimator estimator;
  orientation_estimator_init(&estimator, MADGWICK_BETA_DEFAULT, 0.98f);

  for (int i = 0; i < 200; i++) {
    EXPECT_TRUE(orientation_update_marg(
        &estimator, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 0.01f));
  }

  float roll;
  float pitch;
  float yaw;
  EXPECT_TRUE(orientation_get_euler_rad(&estimator, &roll, &pitch, &yaw));
  EXPECT_NEAR(roll, 0.0f, 1.0e-5f);
  EXPECT_NEAR(pitch, 0.0f, 1.0e-5f);
  EXPECT_NEAR(yaw, 0.0f, 1.0e-5f);
  EXPECT_NEAR(estimator.complementary.roll_rad, 0.0f, 1.0e-5f);
  EXPECT_NEAR(estimator.complementary.pitch_rad, 0.0f, 1.0e-5f);
  EXPECT_NEAR(estimator.complementary.yaw_rad, 0.0f, 1.0e-5f);
}

static void test_imu_yaw_is_gyro_integrated(void) {
  struct orientation_estimator estimator;
  orientation_estimator_init(&estimator, 0.0f, 1.0f);

  for (int i = 0; i < 100; i++) {
    EXPECT_TRUE(orientation_update_imu(
        &estimator, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, PI_F / 2.0f, 0.01f));
  }

  float roll;
  float pitch;
  float yaw;
  EXPECT_TRUE(orientation_get_euler_rad(&estimator, &roll, &pitch, &yaw));
  EXPECT_NEAR(yaw, PI_F / 2.0f, 2.0e-4f);
  EXPECT_NEAR(estimator.complementary.yaw_rad, PI_F / 2.0f, 2.0e-4f);
}

static void test_relative_orientation(void) {
  const struct quaternion parent = z_rotation(30.0f * PI_F / 180.0f);
  const struct quaternion child = z_rotation(75.0f * PI_F / 180.0f);
  struct quaternion relative;
  float joint_angle;

  EXPECT_TRUE(quaternion_relative(&parent, &child, &relative));
  EXPECT_TRUE(quaternion_twist_angle(&relative, 0.0f, 0.0f, 1.0f,
                                     &joint_angle));
  EXPECT_NEAR(joint_angle, 45.0f * PI_F / 180.0f, 1.0e-6f);

  const struct quaternion x_turn = axis_rotation(1.0f, 0.0f, 0.0f,
                                                  -20.0f * PI_F / 180.0f);
  EXPECT_TRUE(quaternion_twist_angle(&x_turn, 1.0f, 0.0f, 0.0f,
                                     &joint_angle));
  EXPECT_NEAR(joint_angle, -20.0f * PI_F / 180.0f, 1.0e-6f);

  const struct quaternion y_turn = axis_rotation(0.0f, 1.0f, 0.0f,
                                                  60.0f * PI_F / 180.0f);
  EXPECT_TRUE(quaternion_twist_angle(&y_turn, 0.0f, 2.0f, 0.0f,
                                     &joint_angle));
  EXPECT_NEAR(joint_angle, 60.0f * PI_F / 180.0f, 1.0e-6f);
}

static void test_invalid_inputs(void) {
  struct orientation_estimator estimator;
  orientation_estimator_init(&estimator, MADGWICK_BETA_DEFAULT, 0.98f);
  EXPECT_TRUE(!orientation_update_imu(
      &estimator, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f));

  const struct quaternion identity = {.w = 1.0f};
  float angle;
  EXPECT_TRUE(!quaternion_twist_angle(&identity, 0.0f, 0.0f, 0.0f, &angle));
}

int main(void) {
  test_stationary_filters();
  test_imu_yaw_is_gyro_integrated();
  test_relative_orientation();
  test_invalid_inputs();

  if (failures != 0) {
    printf("Orientation tests failed: %d\n", failures);
    return 1;
  }

  printf("All orientation tests passed.\n");
  return 0;
}
