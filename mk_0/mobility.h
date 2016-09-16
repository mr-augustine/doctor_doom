#ifndef _MOBILITY_H_
#define _MOBILITY_H_

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "pins.h"
#include "statevars.h"


#define STEERING_ISR_VECT     TIMER1_COMPA_vect
#define STEERING_COMPARE_REG  OCR1A
#define THROTTLE_ISR_VECT     TIMER1_COMPB_vect
#define THROTTLE_COMPARE_REG  OCR1B

#define FWD_ACCEL_RATE_US     100
#define FWD_TO_STOP_RATE_US   100
#define REV_TO_STOP_RATE_US   100
#define REV_RATE_US           10
#define PRE_REV_STOP_US       1400
#define PRE_REV_HOLD_ITERS    40
#define TNP_MIN_ITERATIONS    100   // minimum number of iterations to execute
                                    // throttle neutral protection bypass

#define SPEED_FWD_CREEP       1600
#define SPEED_FWD_CRUISE      1800
#define SPEED_FWD_LUDICROUS   2000
#define SPEED_NEUTRAL         1500
#define SPEED_REV_CREEP       1400
#define SPEED_REV_CRUISE      1200
#define SPEED_REV_LUDICROUS   1100
#define TURN_FULL_LEFT        1900
#define TURN_FULL_RIGHT       1100
#define TURN_NEUTRAL          1500

typedef enum {
  Speed_Creep,
  Speed_Cruise,
  Speed_Ludicrous
} Drive_Speed;

typedef enum {
  Gear_Forward,
  Gear_Pre_Reverse,
  Gear_Reverse,
  Gear_Neutral
} Drive_Gear;

static uint16_t mobility_throttle_us;
static uint16_t mobility_steer_us;

static uint8_t current_hold_iterations;

Drive_Gear current_gear;

// These Interrupt Service Routines are used to terminate the PWM pulses
// for the steering servo and drive motor. The PWM pulses are terminated
// when the timer reaches the values associated with the desired pulse
// durations. Those values are set in the output compare registers.
ISR(STEERING_ISR_VECT) {
  STEERING_PORT &= ~(1 << STEERING_PIN);
}

ISR(THROTTLE_ISR_VECT) {
  THROTTLE_PORT &= ~(1 << THROTTLE_PIN);
}


uint8_t mobility_init(void);
void mobility_start_control_output(void);
void mobility_drive_fwd(Drive_Speed speed);
void mobility_drive_rev(Drive_Speed speed);
void mobility_hardstop(void);
void mobility_blocking_stop(void);
void mobility_stop(void);
void mobility_steer(uint16_t steer_pwm);

static void tnp_bypass(uint16_t iterations) {
  uint16_t pulse_on_duration_us = 1500;
  uint16_t pulse_off_duration_us = 23500;

  uint8_t pulse_iterations = iterations;
  uint8_t i;

  // Ensure the drive and steering pins are initially low
  THROTTLE_PORT &= ~(1 << THROTTLE_PIN);
  STEERING_PORT &= ~(1 << STEERING_PIN);

  for (i = 0; i < pulse_iterations; i++) {
    // Start the neutral pwm pulses
    THROTTLE_PORT |= (1 << THROTTLE_PIN);
    STEERING_PORT |= (1 << STEERING_PIN);

    // Hold the pulse
    _delay_us(pulse_on_duration_us);

    // end the pulse
    THROTTLE_PORT &= ~(1 << THROTTLE_PIN);
    STEERING_PORT &= ~(1 << STEERING_PIN);
    _delay_us(pulse_off_duration_us);
  }

  return;
}

uint8_t mobility_init(void) {
  current_gear = Gear_Neutral;

  THROTTLE_PORT = 0;
  STEERING_PORT = 0;

  THROTTLE_DDR = (1 << THROTTLE_PIN);
  STEERING_DDR = (1 << STEERING_PIN);

  // Begin Throttle Neutral Protection bypass
  tnp_bypass(TNP_MIN_ITERATIONS);

  mobility_stop();
  mobility_steer(TURN_NEUTRAL);

  return 1;
}

void mobility_start_control_output(void) {
  THROTTLE_PORT |= (1 << THROTTLE_PIN);
  STEERING_PORT |= (1 << STEERING_PIN);

  // TODO Really, you should only set the output compare enable bits here.
  // (bits 1 and 2). Leave bit 0 alone.
  TIMSK1 = 0b00000111;  // Enable output compare timers to trigger

  return;
}

void mobility_drive_fwd(Drive_Speed speed) {
  if (current_gear == Gear_Forward || current_gear == Gear_Neutral) {
    uint16_t target_speed_us = SPEED_NEUTRAL;

    switch (speed) {
      case Speed_Creep:
        target_speed_us = SPEED_FWD_CREEP;
        break;
      case Speed_Cruise:
        target_speed_us = SPEED_FWD_CRUISE;
        break;
      case Speed_Ludicrous:
        target_speed_us = SPEED_FWD_LUDICROUS;
        break;
      default:
        mobility_stop();
        return;
    }

    // Ramp up the speed if we're going slower that our target speed. Otherwise
    // just apply the target speed. The objective here is to have the robot
    // throttle up smoothly.
    if (mobility_throttle_us < target_speed_us) {
      mobility_throttle_us += FWD_ACCEL_RATE_US;

      // If we overshot the target speed, then reduce
      if (mobility_throttle_us > target_speed_us) {
        mobility_throttle_us = target_speed_us;
      }
    } else {
      mobility_throttle_us = target_speed_us;
    }

    // Update the gear to forward in case we entered this function while neutral
    current_gear = Gear_Forward;

    THROTTLE_COMPARE_REG = mobility_throttle_us >> 2;

    statevars.mobility_motor_pwm = THROTTLE_COMPARE_REG;
  }

  // NOTE: We're not handling the case where the robot is driving in reverse.
  // That case is outside of this function's scope.
}

void mobility_drive_rev(Drive_Speed speed) {
  // If you've already completed the reverse init
  // Identify the PWM that corresponds with the specified speed
  if (current_gear == Gear_Reverse) {
    uint16_t target_speed_us = SPEED_NEUTRAL;

    switch(speed) {
      case Speed_Creep:
        target_speed_us = SPEED_REV_CREEP;
        break;
      case Speed_Cruise:
        target_speed_us = SPEED_REV_CRUISE;
        break;
      case Speed_Ludicrous:
        target_speed_us = SPEED_REV_LUDICROUS;
        break;
      default:
        // target_speed_us will be neutral
        // TODO consider reporting an error in this case
        ;
    }

    //
    if (mobility_throttle_us > target_speed_us) {
      mobility_throttle_us -= REV_RATE_US;
    }
    // Note: if you're trying to reduce your reverse speed, you will immediately
    // adjust the PWM signal to the target speed. We're more concerned about
    // ramping up the reverse speed gradually.
    else {
      mobility_throttle_us = target_speed_us;
    }
    // continue decreasing PWM until you hit Drive_Speed
  }

  // If you haven't completed the reverse init
  else {
    switch (current_gear) {
      case Gear_Forward:
        // TODO Consider a different way of handling this. This code might cause
        // stress to the drive gear because the robot wouldn't have stopped for
        // very long before reversing directions. On the other hand, we might
        // be relying on the Electronic Speed Control to prevent immediate
        // reversal until the pre-reverse is performed.
        mobility_stop();
        return;
        break;
      case Gear_Neutral:
        current_gear = Gear_Pre_Reverse;
        mobility_throttle_us -= REV_RATE_US;
        break;
      case Gear_Pre_Reverse:
        if (mobility_throttle_us > PRE_REV_STOP_US) {
          mobility_throttle_us -= REV_RATE_US;
          current_hold_iterations = 0;
        } else {
          // Setting the throtte in case the REV_RATE_US and PWM values don't
          // create a value expressed in hundreds of microseconds
          mobility_throttle_us = PRE_REV_STOP_US;

          // continue to hold the pre_reverse_stop signal until counter expires
          if (current_hold_iterations < PRE_REV_HOLD_ITERS) {
            current_hold_iterations++;
          } else {
            current_gear = Gear_Reverse;
            mobility_throttle_us = SPEED_NEUTRAL;
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

  THROTTLE_COMPARE_REG = mobility_throttle_us >> 2;

  statevars.mobility_motor_pwm = THROTTLE_COMPARE_REG;

  return;
}

/* Commands the drive motor and steering servo to be in neutral by sending
 * a neutral pulse to both. The pulse occupies 1/40th of a second from start
 * to finish.
 * Note: This function should not be called within the main loop because it
 * prevents other operations from occuring (except interrupts).
 */
void mobility_blocking_stop(void) {
  tnp_bypass(1);

  return;
}

// TODO This function probably wouldn't produce the desired effect. The objective
// is to force a rapid deceleration, not an immediate coast.
void mobility_hardstop(void) {
  mobility_throttle_us = SPEED_NEUTRAL;

  THROTTLE_COMPARE_REG = SPEED_NEUTRAL >> 2;

  statevars.mobility_motor_pwm = THROTTLE_COMPARE_REG;

  return;
}

void mobility_stop(void) {
  switch (current_gear) {

    // If you're already in neutral
    case Gear_Neutral:
      mobility_throttle_us = SPEED_NEUTRAL;
      break;

    // If you're currently driving forward and want to stop, decrease the PWM signal
    // by a certain amount for this iteration. Continue doing so for subsequent
    // iterations until the PWM width equals 1500. Then change the current_gear
    // to Gear_Neutral.
    case Gear_Forward:
      if (mobility_throttle_us > SPEED_NEUTRAL) {
        mobility_throttle_us -= FWD_TO_STOP_RATE_US;
      } else {
        mobility_throttle_us = SPEED_NEUTRAL;
        current_gear = Gear_Neutral;
      }
      break;

    // Likewise, if you're currently driving in reverse and want to stop, increase
    // the PWM signal by a certain amount for this iteration. Continue doing so
    // until the PWM width equals 1500. Then change the current_gear to
    // Gear_Neutral.
    case Gear_Reverse:
      if (mobility_throttle_us < SPEED_NEUTRAL) {
        mobility_throttle_us += REV_TO_STOP_RATE_US;
      } else {
        mobility_throttle_us = SPEED_NEUTRAL;
        current_gear = Gear_Neutral;
      }
      break;

    default:
      // TODO consider reporting an error in this case
      ;
  }

  THROTTLE_COMPARE_REG = mobility_throttle_us >> 2;

  statevars.mobility_motor_pwm = THROTTLE_COMPARE_REG;

  return;
}

void mobility_steer(uint16_t steer_pwm) {
  // Steering values greater than 1500 will steer Left. And steering values
  // less than 1500 will steer Right. This code snippet prevents the
  // steering servo from getting a signal that would command it beyond its
  // turn limits.
  if (steer_pwm < TURN_FULL_RIGHT) {
    steer_pwm = TURN_FULL_RIGHT;
  }

  if (steer_pwm > TURN_FULL_LEFT) {
    steer_pwm = TURN_FULL_LEFT;
  }

  mobility_steer_us = steer_pwm;

  STEERING_COMPARE_REG = mobility_steer_us >> 2;

  statevars.mobility_steering_pwm = STEERING_COMPARE_REG;

  return;
}

#endif
