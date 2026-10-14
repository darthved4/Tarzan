#ifndef TARZAN_PID_H
#define TARZAN_PID_H

#include <stdbool.h>

/*
 * PID controller contract for arm joint-position control:
 *
 *   setpoint, measurement  joint position (rad)
 *   dt_seconds             elapsed time (s)
 *   output                 desired joint velocity (rad/s)
 *
 * The implementation itself is unit-agnostic as long as callers use one
 * consistent unit system.
 */
struct pid_config {
  float kp;
  float ki;
  float kd;

  float output_min;
  float output_max;

  /* Limits on the integral contribution to the output, not on error*time. */
  float integral_min;
  float integral_max;

  /* First-order derivative-filter time constant in seconds. Zero disables
   * filtering. */
  float derivative_filter_tau;
};

struct pid_controller {
  struct pid_config config;

  float previous_measurement;
  float filtered_measurement_derivative;

  /* Last valid update diagnostics. */
  float error;
  float p_term;
  float i_term;
  float d_term;
  float output;

  bool has_previous_measurement;
  bool config_valid;
};

/*
 * Initialize a controller and clear its dynamic state. A configuration is
 * valid when all fields are finite, gains and derivative_filter_tau are
 * non-negative, ordered limits contain zero, and minimums do not exceed
 * maximums. An invalid configuration leaves the controller safely disabled.
 */
void pid_init(struct pid_controller *pid,
              const struct pid_config *config);

/* Clear dynamic state and diagnostics while preserving the configuration. */
void pid_reset(struct pid_controller *pid);

/*
 * Run one PID update. Derivative action is applied to measurement rather than
 * error, preventing derivative kick on setpoint changes.
 *
 * Invalid controller/input/dt or non-finite intermediate arithmetic returns
 * 0.0f without changing controller state. Callers must use this return value
 * as the command; pid->output remains a diagnostic from the last valid update.
 */
float pid_update(struct pid_controller *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds);

#endif /* TARZAN_PID_H */
