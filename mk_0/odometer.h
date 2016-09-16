#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include <avr/interrupt.h>
#include <avr/io.h>

#include "pins.h"
#include "statevars.h"

#define ODOMETER_ISR_VECT             INT2_vect
#define ODOMETER_INTERRUPT_MASK_PIN   INT2

typedef enum {
  Direction_Forward,
  Direction_Reverse
} Wheel_Direction;

static volatile uint32_t fwd_count;
static volatile uint32_t rev_count;
static volatile uint32_t tick_time;
static Wheel_Direction wheel_turn_direction;

static void initialize_odometer_statevars(void);

uint8_t odometer_init(void);
void odometer_reset(void);
void odometer_set_direction(Wheel_Direction wd);
uint32_t odometer_get_fwd_count(void);
uint32_t odometer_get_rev_count(void);
uint32_t odometer_get_tick_time(void);
void odometer_update(void);

ISR(ODOMETER_ISR_VECT) {
  // TODO Decide if we should also grab a timestamp during this event.
  // Taking a timestamp reading closer to the source should be more accurate.
  // Though I'm not sure by how much. ISRs really should be lean.
  // Update: according to data collected in a previous test, you can expect
  // to see anywhere from zero up to four ticks in a single iteration.

  // Increment the approriate count variable (fwd_count or rev_count)
  if (wheel_turn_direction == Direction_Forward) {
    fwd_count++;
    //Serial.println(fwd_count);
  } else {
    rev_count++;
  }

  // The micros() function doesn't appear to produce meaningful results. Many
  // times, the micros value is repeated between iterations, and those values
  // are unexpectedly low. This time we'll try using the main loop timer (TCNT1)
  // to get timing information. Each tick represents 4 microseconds. And we can
  // expect up to 25,000 microseconds in an iteration.
  //tick_time = micros();
  tick_time = TCNT1;
}

static void initialize_odometer_statevars(void) {
  statevars.odometer_ticks = 0.0;
  statevars.odometer_timestamp = 0;
  statevars.odometer_ticks_are_fwd = 1;

  return;
}

uint8_t odometer_init(void) {
  // Turn on the pull-up resistor for the odometer pin
  ODOMETER_PORT |= (1 << ODOMETER_PIN);

  // Set the odometer pin as an input
  ODOMETER_DDR &= ~(1 << ODOMETER_PIN);

  // Set the External Interrupt to Falling Edge
  // The odometer pin is normally high when the magnet is not present, and then
  // becomes low when the magnet passes in front of it.
  // See Table 15-1 in the Atmel specs
  EICRA &= ~(1 << ISC20);
  EICRA |= (1 << ISC21);

  // Enable interrupts on odometer pin
  EIMSK |= (1 << ODOMETER_INTERRUPT_MASK_PIN);

  odometer_reset();
  odometer_set_direction(Direction_Forward);
  initialize_odometer_statevars();

  return 1;
}

void odometer_reset(void) {
  fwd_count = 0;
  rev_count = 0;
  tick_time = 0;

  return;
}

void odometer_set_direction(Wheel_Direction wd) {
  wheel_turn_direction = wd;

  return;
}

uint32_t odometer_get_fwd_count(void) {
  return fwd_count;
}

uint32_t odometer_get_rev_count(void) {
  return rev_count;
}

uint32_t odometer_get_tick_time(void) {
  return tick_time;
}

void odometer_update(void) {
  // TODO consider copying the timestamp that was (will be) set by the ISR
  // The ISR will probably write to a volatile variable; just copy that value
  // into the statevars.timestamp field. This will represent the final timestamp
  // of the final tick during the current iteration.
  if (wheel_turn_direction == Direction_Forward) {
    statevars.odometer_ticks = fwd_count;
    statevars.odometer_ticks_are_fwd = 1;
  } else {
    statevars.odometer_ticks = rev_count;
    statevars.odometer_ticks_are_fwd = 0;
  }

  statevars.odometer_timestamp = tick_time;

  // reset the tick_time for this iteration
  tick_time = 0;

  return;
}

#endif
