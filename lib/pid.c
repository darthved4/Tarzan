#include <Tarzan/lib/pid.h>

#include <math.h>
#include <stddef.h>

static float clampf(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

static bool config_is_valid(const struct pid_config *config) {
  if (config == NULL) {
    return false;
  }

  if (!isfinite(config->kp) || !isfinite(config->ki) ||
      !isfinite(config->kd) || !isfinite(config->output_min) ||
      !isfinite(config->output_max) ||
      !isfinite(config->integral_min) ||
      !isfinite(config->integral_max) ||
      !isfinite(config->derivative_filter_tau)) {
    return false;
  }

  if (config->kp < 0.0f || config->ki < 0.0f || config->kd < 0.0f ||
      config->derivative_filter_tau < 0.0f) {
    return false;
  }

  if (config->output_min > 0.0f || config->output_max < 0.0f ||
      config->output_min > config->output_max) {
    return false;
  }

  if (config->integral_min > 0.0f || config->integral_max < 0.0f ||
      config->integral_min > config->integral_max) {
    return false;
  }

  return true;
}

void pid_init(struct pid_controller *pid,
              const struct pid_config *config) {
  if (pid == NULL) {
    return;
  }

  *pid = (struct pid_controller){0};

  if (!config_is_valid(config)) {
    return;
  }

  pid->config = *config;
  pid->config_valid = true;
}

void pid_reset(struct pid_controller *pid) {
  if (pid == NULL) {
    return;
  }

  pid->previous_measurement = 0.0f;
  pid->filtered_measurement_derivative = 0.0f;
  pid->error = 0.0f;
  pid->p_term = 0.0f;
  pid->i_term = 0.0f;
  pid->d_term = 0.0f;
  pid->output = 0.0f;
  pid->has_previous_measurement = false;
}

float pid_update(struct pid_controller *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds) {
  if (pid == NULL || !pid->config_valid || !isfinite(setpoint) ||
      !isfinite(measurement) || !isfinite(dt_seconds) ||
      dt_seconds <= 0.0f) {
    return 0.0f;
  }

  const struct pid_config *config = &pid->config;
  const float error = setpoint - measurement;

  if (!isfinite(error)) {
    return 0.0f;
  }

  const float p_term = config->kp * error;
  const float integral_increment = config->ki * error * dt_seconds;

  if (!isfinite(p_term) || !isfinite(integral_increment)) {
    return 0.0f;
  }

  float filtered_measurement_derivative = 0.0f;

  if (pid->has_previous_measurement) {
    const float raw_measurement_derivative =
        (measurement - pid->previous_measurement) / dt_seconds;

    if (!isfinite(raw_measurement_derivative)) {
      return 0.0f;
    }

    float filter_alpha = 1.0f;
    if (config->derivative_filter_tau > 0.0f) {
      const float filter_denominator =
          config->derivative_filter_tau + dt_seconds;
      if (!isfinite(filter_denominator)) {
        return 0.0f;
      }
      filter_alpha = dt_seconds / filter_denominator;
    }

    filtered_measurement_derivative =
        pid->filtered_measurement_derivative +
        filter_alpha *
            (raw_measurement_derivative -
             pid->filtered_measurement_derivative);
  }

  if (!isfinite(filtered_measurement_derivative)) {
    return 0.0f;
  }

  const float d_term = -config->kd * filtered_measurement_derivative;
  float i_term = clampf(pid->i_term + integral_increment,
                        config->integral_min, config->integral_max);

  if (!isfinite(d_term) || !isfinite(i_term)) {
    return 0.0f;
  }

  float unclamped_output = p_term + i_term + d_term;
  if (!isfinite(unclamped_output)) {
    return 0.0f;
  }

  /* Conditional integration anti-windup: reject only the proposed integral
   * change that would drive a saturated output farther into saturation.
   * Integral changes that unwind saturation remain active. */
  const bool drives_further_high =
      unclamped_output > config->output_max && i_term > pid->i_term;
  const bool drives_further_low =
      unclamped_output < config->output_min && i_term < pid->i_term;

  if (drives_further_high || drives_further_low) {
    i_term = pid->i_term;
    unclamped_output = p_term + i_term + d_term;
  }

  if (!isfinite(unclamped_output)) {
    return 0.0f;
  }

  const float output = clampf(unclamped_output, config->output_min,
                              config->output_max);

  /* Commit state only after the complete update has been validated. */
  pid->previous_measurement = measurement;
  pid->filtered_measurement_derivative =
      filtered_measurement_derivative;
  pid->error = error;
  pid->p_term = p_term;
  pid->i_term = i_term;
  pid->d_term = d_term;
  pid->output = output;
  pid->has_previous_measurement = true;

  return output;
}
