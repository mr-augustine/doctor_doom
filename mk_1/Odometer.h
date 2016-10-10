/*
 * file: Odometer.h
 * created: 20161010
 * author(s): mr-augustine
 *
 * Describes the class used to initialize and update the odometer.
 */
#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Pins.h"
#include "Statevars.h"

#define ODOMETER_ISR_VECT             INT2_vect
#define ODOMETER_INTERRUPT_MASK_PIN   INT2

enum WheelDirection {
  WheelDir_Forward = 0,
  WheelDir_Reverse,
};

class Odometer {
private:
  Statevars * vars;


  void initialize_odometer_statevars(void);

public:
  Odometer(const Statevars * s);
  void initialize(void);
  int8_t verify_init(void);
  void reset(void);
  void set_direction(WheelDirection wd);
  uint32_t get_fwd_count(void);
  uint32_t get_rev_count(void);
  uint32_t get_tick_time(void);
  void update(void);

};

#endif // #ifndef _ODOMETER_H_
