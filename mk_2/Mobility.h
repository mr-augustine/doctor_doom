/*
 * file: Mobility.h
 * created: 20161009
 * author(s): mr-augustine
 *
 * Describes the class used to initialize and control the steering servo and
 * drive motor.
 */
#ifndef _MOBILITY_H_
#define _MOBILITY_H_

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>

#include "Pins.h"
#include "Statevars.h"

#define MOBILITY_PWM_PERIOD_MS    20  // 50 Hz PWM pulses -> 1/50 = 20 ms
#define MILLISEC_PER_SEC          1000
#define MICROSEC_PER_MILLISEC     1000
#define MOBILITY_TIMER_PRESCALER  ((1 << CS30) | (1 << CS31)) // prescaler = 64
#define PRESCALED_TICKS_PER_SEC   250000

#define FWD_ACCEL_RATE_US         100
#define FWD_TO_STOP_RATE_US       100
#define REV_TO_STOP_RATE_US       100
#define REV_RATE_US               10
#define PRE_REV_STOP_US           1400
#define PRE_REV_HOLD_ITERS        40
#define TNP_MIN_ITERATIONS        100   // minimum number of iterations to execute
                                        // throttle neutral protection bypass

#define THROT_FWD_CREEP           1600
#define THROT_FWD_CRUISE          1800
#define THROT_FWD_LUDICROUS       2000
#define THROT_NEUTRAL             1500
#define THROT_REV_CREEP           1400
#define THROT_REV_CRUISE          1200
#define THROT_REV_LUDICROUS       1100
#define STEER_FULL_LEFT           1900
#define STEER_FULL_RIGHT          1100
#define STEER_NEUTRAL             1500

enum DriveSpeed {
  DriveSpeed_Creep = 0,
  DriveSpeed_Cruise,
  DriveSpeed_Ludicrous,
};

enum DriveGear {
  DriveGear_Forward = 0,
  DriveGear_PreReverse,
  DriveGear_Reverse,
  DriveGear_Neutral,
};

class Mobility {
private:
  uint16_t throttle_us;
  uint16_t steering_us;
  DriveGear current_gear;
  uint8_t current_hold_iterations;
  Statevars * vars;

  uint16_t pwm_to_ticks(uint16_t pwm_us);
  void tnp_bypass(uint16_t iterations);

public:
  const uint32_t ticks_per_period = PRESCALED_TICKS_PER_SEC / (MILLISEC_PER_SEC / MOBILITY_PWM_PERIOD_MS);
  const uint32_t microsec_per_period = MICROSEC_PER_MILLISEC * MOBILITY_PWM_PERIOD_MS;

  Mobility(const Statevars * s);
  void initialize(void);
  int8_t verify_init(void);
  void drive_fwd(DriveSpeed speed);
  void drive_rev(DriveSpeed speed);
  void blocking_stop(void);
  void steer(uint16_t steer_pwm);
  void stop(void);
};

#endif
