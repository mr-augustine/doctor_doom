#include "Control.h"
#include "Mobility.h"
#include "Navigation.h"
#include "Robot.h"

Control::Control(const Statevars * s) {
  vars = s;
}

void Control::initialize(void) {
  xtrack_error = 0.0;
  xtrack_error_prev = 0.0;
  xtrack_error_rate = 0.0;
  xtrack_error_sum = 0.0;
  steer_control = STEER_NEUTRAL;

  return;
}

int8_t Control::verify_init(void) {
  // Verify that the steering control is initialized to neutral
  if (steer_control != STEER_NEUTRAL) {
    return 0;
  }

  return 1;
}

void Control::update(void) {
  update_xtrack_error();
  update_xtrack_error_rate();
  update_xtrack_error_sum();

  steer_control = (K_PROP * xtrack_error) +
                  (K_RATE * xtrack_error_rate) +
                  (K_INTEGRAL * xtrack_error_sum);

  // Limit validation for steer_control will be handled by the mobility library.
  // This way we don't command the robot to turn beyond the full left/right
  // steering angles

  // If the xtrack_error is NEGATIVE, then the robot is towards the RIGHT of
  // where it needs to be; If the xtrack error is POSITIVE, then the robot is
  // towards the LEFT of where it needs to be. So, if I'm towards the right of
  // the target heading, I need to turn left. To turn left, you increase the
  // steering PWM value. This is why we have a subtraction in the line below.
  vars->set_control_steering_pwm( (uint16_t)(STEER_NEUTRAL - steer_control) );

  return;
}

void Control::update_xtrack_error(void) {
  // Error = Reference Value - Measured value
  xtrack_error_prev = xtrack_error;

  xtrack_error = Navigation::calc_relative_bearing(TARGET_HEADING, vars->get_nav_heading_deg());

  // TODO Change this assignment when you start navigating to waypoints.
  // Normally the desired heading will be the waypt_true_bearing unless we set
  // a new bearing if an obstacle is detected
  vars->set_control_heading_desired(TARGET_HEADING);
  // vars->set_control_heading_desired(waypt_true_bearing);

  vars->set_control_xtrack_error(xtrack_error);
  // TODO Normally the cross track error will be the relative bearing (to the
  // waypoint) unless we decide to set a new bearing if an obstacle is detected
  // vars->set_control_xtrack_error(rel_bearing_deg);

  return;
}

void Control::update_xtrack_error_rate(void) {
  // Rate = (Error - Error_Previous) / Computation Interval
  xtrack_error_rate = (xtrack_error - xtrack_error_prev) / MAIN_LOOP_PERIOD_SEC;

  vars->set_control_xtrack_error_rate(xtrack_error_rate);

  return;
}

void Control::update_xtrack_error_sum(void) {
  // TODO Be sure to reset the cross track error sum  to zero when the
  // cross track error becomes zero (or very close to it)

  // Rate Sum = Rate Sum + Error * Computation Interval
  xtrack_error_sum = xtrack_error_sum + xtrack_error * MAIN_LOOP_PERIOD_SEC;

  vars->set_control_xtrack_error_sum(xtrack_error_sum);

  return;
}
