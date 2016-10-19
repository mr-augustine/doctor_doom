/*
 * file: Odometer.h
 * created: 20161010
 * author(s): mr-augustine
 */
#include "Odometer.h"

volatile uint32_t fwd_count;
volatile uint32_t rev_count;
volatile uint32_t tick_time;

WheelDirection wheel_turn_direction;

ISR(ODOMETER_ISR_VECT) {
  // TODO Decide if we should also grab a timestamp during this event.
  // Taking a timestamp reading closer to the source should be more accurate.
  // Though I'm not sure by how much. ISRs really should be lean.
  // Update: according to data collected in a previous test, you can expect
  // to see anywhere from zero up to four ticks in a single iteration.

  // Increment the approriate count variable (fwd_count or rev_count)
  if (wheel_turn_direction == WheelDir_Forward) {
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

Odometer::Odometer(const Statevars * s) {
  vars = s;
}

void Odometer::initialize(void) {
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

  reset();
  set_direction(WheelDir_Forward);
  initialize_odometer_statevars();

  return;
}

int8_t Odometer::verify_init(void) {
  // Verify that the pull-up resistor is enabled for the odometer pin
  if ( !(ODOMETER_PORT & (1 << ODOMETER_PIN)) ) {
    return 0;
  }

  // Verify that the odometer pin is set as an input pin
  if ( (ODOMETER_DDR & (1 << ODOMETER_PIN)) ) {
    return -1;
  }

  // Verify that the External Interrupt is set to Falling Edge
  if ( (EICRA & (1 << ISC20)) ||
       !(EICRA & (1 << ISC21)) ) {
    return -2;
  }

  // Verify interrupts are enabled for the odometer pin
  if ( !(EIMSK & (1 << ODOMETER_INTERRUPT_MASK_PIN)) ) {
    return -3;
  }

  return 1;
}

uint32_t Odometer::get_fwd_count(void) {
  return fwd_count;
}

uint32_t Odometer::get_rev_count(void) {
  return rev_count;
}

uint32_t Odometer::get_tick_time(void) {
  return tick_time;
}

void Odometer::reset(void) {
  fwd_count = 0;
  rev_count = 0;
  tick_time = 0;

  return;
}

void Odometer::initialize_odometer_statevars(void) {
  vars->set_odometer_ticks(0.0);
  vars->set_odometer_timestamp(0);
  vars->set_odometer_ticks_are_fwd(1);

  return;
}

void Odometer::set_direction(WheelDirection wd) {
  wheel_turn_direction = wd;

  return;
}

void Odometer::update(void) {
  // TODO consider copying the timestamp that was (will be) set by the ISR
  // The ISR will probably write to a volatile variable; just copy that value
  // into the statevars.timestamp field. This will represent the final timestamp
  // of the final tick during the current iteration.
  if (wheel_turn_direction == WheelDir_Forward) {
    vars->set_odometer_ticks(fwd_count);
    vars->set_odometer_ticks_are_fwd(1);
  } else {
    vars->set_odometer_ticks(rev_count);
    vars->set_odometer_ticks_are_fwd(0);
  }

  vars->set_odometer_timestamp(tick_time);

  // reset the tick_time for this iteration
  tick_time = 0;

  return;
}
