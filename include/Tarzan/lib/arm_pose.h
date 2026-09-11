#ifndef TARZAN_ARM_POSE_H
#define TARZAN_ARM_POSE_H

#include <Tarzan/lib/link_orientation.h>

struct arm_joint_angles {
  float turntable;
  float first_link;
  float second_link;
  float wrist_pitch;
  float wrist_roll;
};

struct arm_pose_config {
  const struct device *first_link_imu;
  const struct device *second_link_imu;
  const struct device *end_effector_imu;
  const struct device *turntable_mag;
  const struct device *base_reference_mag;
  const struct device *end_effector_mag;
  struct sensor_axis_map first_link_map;
  struct sensor_axis_map second_link_map;
  struct sensor_axis_map end_effector_imu_map;
  struct sensor_axis_map end_effector_mag_map;
  float madgwick_beta;
  float complementary_alpha;
};

struct arm_pose_state {
  struct link_orientation first_link;
  struct link_orientation second_link;
  struct link_orientation end_effector;
  const struct device *turntable_mag;
  const struct device *base_reference_mag;
  struct arm_joint_angles madgwick;
  struct arm_joint_angles complementary;
  struct arm_joint_angles madgwick_zero;
  struct arm_joint_angles complementary_zero;
  bool zeroed;
};

int arm_pose_init(struct arm_pose_state *state,
                  const struct arm_pose_config *config);
int arm_pose_calibrate_gyros(struct arm_pose_state *state, uint16_t samples,
                             uint16_t sample_delay_ms);
int arm_pose_update(struct arm_pose_state *state, float dt_seconds);
void arm_pose_zero(struct arm_pose_state *state);

#endif /* TARZAN_ARM_POSE_H */
