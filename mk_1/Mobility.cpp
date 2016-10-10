/*
 * file: Mobility.cpp
 * created: 20161009
 * author(s): mr-augustine
 */
#include "Mobility.h"

Mobility::Mobility(const Statevars * statevars) {
  vars = statevars;

  throttle_us = THROT_NEUTRAL;
  steering_us = STEER_NEUTRAL;
  current_gear = DriveGear_Neutral;
  current_hold_iterations = 0;
}

void Mobility::initialize(void) {
  THROTTLE_PORT = 0;
  STEERING_PORT = 0;

  THROTTLE_DDR |= (1 << THROTTLE_DDR_PIN);
  STEERING_DDR |= (1 << STEERING_DDR_PIN);

  // Even though the steering and throttle pins will most likely be
  // attached to the same timer, we'll handle them separately just in
  // case we decide to put them on separate timers later.
  MOBILITY_TCCRA = 0;
  MOBILITY_TCCRB = 0;
  MOBILITY_TIMER = 0;

  // Specify Fast PWM (see Table 17-2)
  MOBILITY_TCCRA |= (1 << WGM30) | (1 << WGM31);
  MOBILITY_TCCRB |= (1 << WGM32) | (1 << WGM33);

  // Clear the steering and throttle pins when the output compare register
  // matches
  MOBILITY_TCCRA |= (STEERING_COMP_MODE | THROTTLE_COMP_MODE);

  // Set the prescaler (see Table 17-6)
  MOBILITY_TCCRB |= PRESCALER_64;

  // Set the TOP values
  MOBILITY_TOP = ticks_per_period;

  STEERING_COMPARE_REG = pwm_to_ticks(steering_us);
  THROTTLE_COMPARE_REG = pwm_to_ticks(throttle_us);

  tnp_bypass();
}

void Mobility::drive_fwd(DriveSpeed speed) {
  if (current_gear == DriveGear_Forward || current_gear == DriveGear_Neutral) {
    uint16_t target_speed_us = THROT_NEUTRAL;

    switch (speed) {
      case DriveSpeed_Creep:
        target_speed_us = THROT_FWD_CREEP;
        break;
      case DriveSpeed_Cruise:
        target_speed_us = THROT_FWD_CRUISE;
        break;
      case DriveSpeed_Ludicrous:
        target_speed_us = THROT_FWD_LUDICROUS;
        break;
      default:
        stop();
        return;
    }

    // Ramp up the speed if we're going slower that our target speed. Otherwise
    // just apply the target speed. The objective here is to have the robot
    // throttle up smoothly.
    if (throttle_us < target_speed_us) {
      throttle_us += FWD_ACCEL_RATE_US;

      // If we overshot the target speed, then reduce
      if (throttle_us > target_speed_us) {
        throttle_us = target_speed_us;
      }
    } else {
      throttle_us = target_speed_us;
    }

    // Update the gear to forward in case we entered this function while neutral
    current_gear = DriveGear_Forward;

    THROTTLE_COMPARE_REG = pwm_to_ticks(throttle_us);

    vars->set_mobility_motor_pwm(THROTTLE_COMPARE_REG);
  }

  // NOTE: We're not handling the case where the robot is driving in reverse.
  // That case is outside of this function's scope.
  return;
}

void Mobility::drive_rev(DriveSpeed speed) {
  // If you've already completed the reverse init
  // Identify the PWM that corresponds with the specified speed
  if (current_gear == DriveGear_Reverse) {
    uint16_t target_speed_us = THROT_NEUTRAL;

    switch (speed) {
      case DriveSpeed_Creep:
        target_speed_us = THROT_REV_CREEP;
        break;
      case DriveSpeed_Cruise:
        target_speed_us = THROT_REV_CRUISE;
        break;
      case DriveSpeed_Ludicrous:
        target_speed_us = THROT_REV_LUDICROUS;
        break;
      default:
        // target_speed_us will be neutral
        // TODO consider reporting an error in this case
        stop();
        return;
    }

    // continue decreasing PWM until you hit Drive_Speed
    if (throttle_us > target_speed_us) {
      throttle_us -= REV_RATE_US;
    }
    // NOTE: if you're trying to reduce your reverse speed, you will immediately
    // adjust the PWM signal to the target speed. We're more concerned about
    // ramping up the reverse speed gradually.
    else {
      throttle_us = target_speed_us;
    }
  }

  // If you haven't completed the reverse init
  else {
    switch (current_gear) {
      case DriveGear_Forward:
        // TODO Consider a different way of handling this. This code might cause
        // stress to the drive gear because the robot wouldn't have stopped for
        // very long before reversing directions. On the other hand, we might
        // be relying on the Electronic Speed Control to prevent immediate
        // reversal until the pre-reverse is performed.
        stop();
        return;
        break;
      case DriveGear_Neutral:
        current_gear = DriveGear_PreReverse;
        throttle_us -= REV_RATE_US;
        break;
      case DriveGear_PreReverse:
        if (throttle_us > PRE_REV_STOP_US) {
          throttle_us -= REV_RATE_US;
          current_hold_iterations = 0;
        } else {
          // Setting the throtte in case the REV_RATE_US and PWM values don't
          // create a value expressed in hundreds of microseconds
          throttle_us = PRE_REV_STOP_US;

          // continue to hold the pre_reverse_stop signal until counter expires
          if (current_hold_iterations < PRE_REV_HOLD_ITERS) {
            current_hold_iterations++;
          } else {
            current_gear = DriveGear_Reverse;
            throttle_us = THROT_NEUTRAL;
            current_hold_iterations = 0;
          }
        }
        break;
      default:
        // TODO consider reporting an error in this case
        ;
    }
    // If you're Gear_Forward, call mobility_stop()
    // Else If you're Gear_Neutral, start Driving_Pre_Reverse
    // Else if you're Driving_Pre_Reverse, continue decreasing PWM until you hit stop, then wait
  }

  THROTTLE_COMPARE_REG = pwm_to_ticks(throttle_us);

  vars->set_mobility_motor_pwm(THROTTLE_COMPARE_REG);

  return;
}

uint16_t Mobility::pwm_to_ticks(uint16_t pwm_us) {
  return (ticks_per_period * pwm_us) / microsec_per_period;
}

void Mobility::steer(uint16_t steer_pwm) {
  // Steering values greater than 1500 will steer Left. And steering values
  // less than 1500 will steer Right. This code snippet prevents the
  // steering servo from getting a signal that would command it beyond its
  // turn limits.
  if (steer_pwm < STEER_FULL_RIGHT) {
    steer_pwm = STEER_FULL_RIGHT;
  }

  if (steer_pwm > STEER_FULL_LEFT) {
    steer_pwm = STEER_FULL_LEFT;
  }

  steering_us = steer_pwm;

  STEERING_COMPARE_REG = pwm_to_ticks(steering_us);

  vars->set_mobility_steering_pwm(STEERING_COMPARE_REG);

  return;
}

void Mobility::stop(void) {
  switch (current_gear) {
    case DriveGear_Neutral:
      throttle_us = THROT_NEUTRAL;
      break;

    // If you're currently driving forward and want to stop, decrease the PWM signal
    // by a certain amount for this iteration. Continue doing so for subsequent
    // iterations until the PWM width equals 1500. Then change the current_gear
    // to Gear_Neutral.
    case DriveGear_Forward:
      if (throttle_us > THROT_NEUTRAL) {
        throttle_us -= FWD_TO_STOP_RATE_US;
      } else {
        throttle_us = THROT_NEUTRAL;
        current_gear = DriveGear_Neutral;
      }
      break;

    // Likewise, if you're currently driving in reverse and want to stop, increase
    // the PWM signal by a certain amount for this iteration. Continue doing so
    // until the PWM width equals 1500. Then change the current_gear to
    // Gear_Neutral.
    case DriveGear_Reverse:
      if (throttle_us < THROT_NEUTRAL) {
        throttle_us += REV_TO_STOP_RATE_US;
      } else {
        throttle_us = THROT_NEUTRAL;
        current_gear = DriveGear_Neutral;
      }
      break;

    default:
      // TODO consider reporting an error in this case
      ;
  }

  THROTTLE_COMPARE_REG = pwm_to_ticks(throttle_us);

  vars->set_mobility_motor_pwm(THROTTLE_COMPARE_REG);

  return;
}

int8_t Mobility::verify_init(void) {
  // Verify that the steering and throttle pin directions are set
  if ( !(THROTTLE_DDR & (1 << THROTTLE_DDR_PIN)) ||
       !(STEERING_DDR & (1 << STEERING_DDR_PIN)) ) {
    return 0;
  }

  // Verify that Fast PWM is set
  if ( !(MOBILITY_TCCRA & (1 << WGM30)) ||
       !(MOBILITY_TCCRA & (1 << WGM31)) ||
       !(MOBILITY_TCCRB & (1 << WGM32)) ||
       !(MOBILITY_TCCRB & (1 << WGM33)) ) {
    return -1;
  }

  // Verify that the compare mode bits are set for the steering and throttle
  if ( !(MOBILITY_TCCRA & STEERING_COMP_MODE) ||
       !(MOBILITY_TCCRA & THROTTLE_COMP_MODE) ) {
    return -2;
  }

  // Verify that the Timer prescaler is set
  if ( !(MOBILITY_TCCRB & PRESCALER_64) ) {
    return -3;
  }

  // Verify that the TOP value is set
  if ( !(MOBILITY_TOP == ticks_per_period) ) {
    return -4;
  }

  // Verify that the initial PWM pulse widths are set
  if ( !(STEERING_COMPARE_REG == pwm_to_ticks(steering_us)) ||
       !(THROTTLE_COMPARE_REG == pwm_to_ticks(throttle_us)) ) {
    return -5;
  }

  return 1;
}
