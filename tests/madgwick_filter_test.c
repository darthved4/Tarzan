#include <Tarzan/lib/madgwick.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static int failures;

#define EXPECT_TRUE(condition)                                                \
  do {                                                                        \
    if (!(condition)) {                                                       \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);          \
      failures++;                                                             \
    }                                                                         \
  } while (false)

#define EXPECT_NEAR(actual, expected, tolerance)                              \
  do {                                                                        \
    const float actual_value = (actual);                                      \
    if (!isfinite(actual_value) ||                                            \
        fabsf(actual_value - (expected)) > (tolerance)) {                     \
      printf("FAIL %s:%d: %s = %.6f, expected %.6f +/- %.6f\n",            \
             __FILE__, __LINE__, #actual, (double)actual_value,               \
             (double)(expected), (double)(tolerance));                        \
      failures++;                                                             \
    }                                                                         \
  } while (false)

static void test_stationary_orientation(void) {
  struct madgwick_state state;
  madgwick_init(&state, MADGWICK_BETA_DEFAULT);

  for (int i = 0; i < 500; i++) {
    EXPECT_TRUE(madgwick_update(&state, 0.0f, 0.0f, 1.0f,
                                0.0f, 0.0f, 0.0f,
                                1.0f, 0.0f, 0.0f, 0.01f));
  }

  float roll;
  float pitch;
  float yaw;
  madgwick_get_euler(&state.orientation, &roll, &pitch, &yaw);
  EXPECT_NEAR(roll, 0.0f, 1.0e-4f);
  EXPECT_NEAR(pitch, 0.0f, 1.0e-4f);
  EXPECT_NEAR(yaw, 0.0f, 1.0e-4f);
}

static void test_gyro_rotation_and_independent_state(void) {
  struct madgwick_state rotating;
  struct madgwick_state stationary;
  madgwick_init(&rotating, 0.0f);
  madgwick_init(&stationary, 0.0f);

  for (int i = 0; i < 100; i++) {
    EXPECT_TRUE(madgwick_update_imu(&rotating, 0.0f, 0.0f, 1.0f,
                                    0.0f, 0.0f, 1.5707963268f, 0.01f));
    EXPECT_TRUE(madgwick_update_imu(&stationary, 0.0f, 0.0f, 1.0f,
                                    0.0f, 0.0f, 0.0f, 0.01f));
  }

  float roll;
  float pitch;
  float yaw;
  madgwick_get_euler(&rotating.orientation, &roll, &pitch, &yaw);
  EXPECT_NEAR(yaw, 90.0f, 0.01f);

  madgwick_get_euler(&stationary.orientation, &roll, &pitch, &yaw);
  EXPECT_NEAR(yaw, 0.0f, 1.0e-4f);
}

static void test_invalid_update_preserves_state(void) {
  struct madgwick_state state;
  madgwick_init(&state, MADGWICK_BETA_DEFAULT);
  const struct quaternion expected = state.orientation;

  EXPECT_TRUE(!madgwick_update_imu(&state, 0.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, 0.0f, 0.01f));
  EXPECT_TRUE(!madgwick_update_imu(&state, 0.0f, 0.0f, 1.0f,
                                   0.0f, 0.0f, 0.0f, 0.0f));
  EXPECT_TRUE(state.orientation.w == expected.w);
  EXPECT_TRUE(state.orientation.x == expected.x);
  EXPECT_TRUE(state.orientation.y == expected.y);
  EXPECT_TRUE(state.orientation.z == expected.z);
}

int main(void) {
  test_stationary_orientation();
  test_gyro_rotation_and_independent_state();
  test_invalid_update_preserves_state();

  if (failures != 0) {
    printf("Madgwick tests failed: %d\n", failures);
    return 1;
  }

  printf("All Madgwick tests passed.\n");
  return 0;
}
