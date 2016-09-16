#include <stdint.h>

#include "doom.h"

#define MAINLOOP_PERIOD_TICKS   6249
#define MISSION_TIMEOUT         3200

statevars_t statevars;
uint32_t iterations;

volatile uint8_t system_timer_overflow = 0;

ISR(TIMER1_OVF_vect) {
  system_timer_overflow = 1;
}

void setup() {

  if (init_all_subsystems()) {
    Serial.println("All systems ready!");
  } else {
    Serial.println("There was a subsystem failure");
    exit(0);
  }

  clear_statevars();
  statevars.prefix = 0xDADAFEED;
  statevars.suffix = 0xCAFEBABE;

  iterations = 0;

  configure_mainloop_timer();
  enable_mainloop_timer();


  Serial.println("Waiting for button to be pressed");
  do {
    led_turn_on();
    button_update();
    mobility_blocking_stop();
  } while (!button_is_pressed());

  if (button_is_pressed()) {
    led_turn_off();
    update_all_inputs();
    update_nav_control_values();
    Serial.println("Mission started!");
  }
}

void loop() {
  TCNT1 = 0;

  // Take note if the system timer (TIMER1) had an overflow
  if (system_timer_overflow) {
    statevars.status |= STATUS_SYS_TIMER_OVERFLOW;
  }

  // Draw the PWM signals one by one before continuing the loop
  mobility_send_steering_pwm(1200);
  mobility_send_throttle_pwm(SPEED_FWD_CREEP);

  // Record the data from the previous iteration
  write_data();

  // Reset statevars and timer overflow flag
  statevars.status = 0;

  statevars.main_loop_counter = iterations;
  system_timer_overflow = 0;

  update_all_inputs();

  update_nav_control_values();

  iterations++;

  // If the button switched to the OFF position, then stop the mission
  if (!button_is_pressed()) {
  // Instead, stop the robot after a certain number of seconds
  //if (iterations > MISSION_TIMEOUT) {
    Serial.println("Finished collecting data!");
    sdcard_finish();

    mobility_blocking_stop();

    exit(0);
  }

  /* Ensure that the main loop period is as long as we want it to be.
   * This means (1) triggering the main loop to restart when we notice it is
   * running too long, and (2) performing busy waiting if the instructions
   * above finish before the desired loop duration.
   */
   if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
     statevars.status |= STATUS_MAIN_LOOP_LATE;

     // Jump to the start of loop() by calling return. Normally we would use
     // continue to go to the beginning of a loop, but in this case, loop() is
     // a function. And when you return from loop() it gets called again by
     // main.cpp.
     return;
   }

  while (1) {
    // System timer reached 250,000
    if (system_timer_overflow) {
      break;
    }

    // Main loop timer reached 6250
    if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
      break;
    }
  }
}

void clear_statevars(void) {
  memset(&statevars, 0, sizeof(statevars));
}

void configure_mainloop_timer(void) {
  cli();

  /* We need to use a timer to control how long our main loop iterations are.
   * Here we are configuring Timer1 because it is a "16-bit" timer, which would
   * allow use to count to values greater than the usual (2^8)-1. For a
   * prescale value of 64, the timer will count up to 16,000,000/64 = 250,000.
   * This gives us a timer period of 250,000 ticks/sec = 4 microseconds/tick.
   * Suppose we want the main loop to run 40 times per second (40 Hz), we would
   * need to restart the loop when Timer1 reaches the value: 250,000/40 = 6250.
   * See Atmel datasheet for Mega, Section 17.11
   */
  TCCR1A = 0b00000000; // Normal operation; no waveform generation by default
  TCCR1B = 0b00000011; // No input capture, waveform gen; prescaler = 64
  TCCR1C = 0b00000000; // No output compare
  TIMSK1 = 0b00000000; // Ensure overflow interrupt is disabled for now

  sei();

  return;
}

void enable_mainloop_timer(void) {
  TIMSK1 = 0b00000001;

  return;
}

uint8_t init_all_subsystems(void) {
  Serial.begin(115200);

  if (!button_init()) {
    Serial.println("LED button couldn't be initialized");
    return 0;
  } else {
    Serial.println("LED button is ready!");
    led_turn_off();
  }

  if (!cmps10_init()) {
    Serial.println("Compass couldn't be initialized");
    return 0;
  } else {
    Serial.println("Compass is ready!");
  }

  if (!gps_init()) {
    Serial.println("GPS sensor couldn't be initialized");
    return 0;
  } else {
    Serial.println("GPS sensor is ready!");
  }

  if (!odometer_init()) {
    Serial.println("Odometer couldn't be initialized");
    return 0;
  } else {
    Serial.println("Odometer is ready!");
  }

  if (!mobility_init()) {
    Serial.println("Mobility couldn't be initialized");
    return 0;
  } else {
    Serial.println("Mobility is ready!");
  }

  if (!sdcard_init(&statevars, sizeof(statevars))) {
    Serial.println("SD card couldn't be initialized");
    return 0;
  } else {
    Serial.println("SD card is ready!");
  }

  return 1;
}
