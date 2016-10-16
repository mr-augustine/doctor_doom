#include "Robot.h"

#define MAINLOOP_PERIOD_TICKS   6249
#define MISSION_TIMEOUT         200

Statevars myVars;
Compass myCompass(&myVars);
Gps myGps(&myVars);
LedButton myButton;
Logger myLogger(&myVars);
Mobility myMobility(&myVars);
Odometer myOdometer(&myVars);
Robot doom(&myVars, &myCompass, &myGps, &myButton, &myOdometer);

uint32_t iterations;

volatile uint8_t system_timer_overflow = 0;

ISR(TIMER1_OVF_vect) {
  system_timer_overflow = 1;
}

void setup() {
  Serial.begin(9600);

  init_all_subsystems();

  if (verify_all_subsystems()) {
    Serial.println("All systems ready!");
  } else {
    Serial.println("There was a subsystem failure");
    exit(0);
  }

  iterations = 0;

  configure_mainloop_timer();
  enable_mainloop_timer();

  Serial.println("Waiting for button to be pressed");
  do {
    myButton.turn_on_led();
    myButton.update();
    myMobility.stop();
  } while (!myButton.is_pressed());

  if (myButton.is_pressed()) {
    myButton.turn_off_led();
    doom.update_all_inputs();
    doom.update_nav_control_values();
    Serial.println("Mission started!");
  }
}

void loop() {
  TCNT1 = 0;

  // Take note if the system timer (TIMER1) had an overflow
  if (system_timer_overflow) {
    myVars.set_status(myVars.get_status() | STATUS_SYS_TIMER_OVERFLOW);
  }

  // Update the PWM widths for the steering and throttle signals
  myMobility.steer(myVars.get_control_steering_pwm());
  myMobility.drive_fwd(DriveSpeed_Creep);

  // Record the data from the previous iteration
  myLogger.write();

  // Reset statevars and timer overflow flag
  myVars.set_status(0);
  myVars.set_main_loop_counter(iterations);
  system_timer_overflow = 0;

  doom.update_all_inputs();
  doom.update_nav_control_values();

  iterations++;

  // If the button switched to the OFF position, then stop the mission
  //if (!button_is_pressed()) {
  // Instead, stop the robot after a certain number of seconds
  if (iterations > MISSION_TIMEOUT) {
    Serial.println("Finished collecting data!");
    myLogger.finish();

    myMobility.stop();

    exit(0);
  }

  /* Ensure that the main loop period is as long as we want it to be.
   * This means (1) triggering the main loop to restart when we notice it is
   * running too long, and (2) performing busy waiting if the instructions
   * above finish before the desired loop duration.
   */
   if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
     myVars.set_status(myVars.get_status() | STATUS_MAIN_LOOP_LATE);

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

}  // end of loop()

void init_all_subsystems(void) {
  cli();

  myVars.initialize();
  myCompass.initialize();
  myGps.initialize();
  myButton.initialize();
  myMobility.initialize();
  myOdometer.initialize();

  sei();

  // The logger needs interrupts enabled so that it can initialize
  myLogger.initialize();

  return;
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

uint8_t verify_all_subsystems(void) {
  int8_t svars_return = myVars.verify_init();
  int8_t cmps_return = myCompass.verify_init();
  int8_t gps_return = myGps.verify_init();
  int8_t button_return = myButton.verify_init();
  int8_t logger_return = myLogger.verify_init();
  int8_t mobility_return = myMobility.verify_init();
  int8_t odo_return = myOdometer.verify_init();

  if (svars_return == 1) {
    Serial.println("Statevars is ready!");
  } else {
    Serial.print("Statevars couldn't be initialized: ");
    Serial.println(svars_return);
    return 0;
  }

  if (cmps_return == 1) {
    Serial.println("Compass is ready!");
  } else {
    Serial.print("Compass couldn't be initialized: ");
    Serial.println(cmps_return);
    return 0;
  }

  if (gps_return == 1) {
    Serial.println("GPS is ready!");
  } else {
    Serial.print("GPS couldn't be initialized: ");
    Serial.println(gps_return);
    return 0;
  }

  if (button_return == 1) {
    Serial.println("LED button is ready!");
  } else {
    Serial.print("LED button couldn't be initialized: ");
    Serial.println(button_return);
    return 0;
  }

  if (mobility_return == 1) {
    Serial.println("Mobility is ready!");
  } else {
    Serial.print("Mobility couldn't be initialized: ");
    Serial.println(mobility_return);
    return 0;
  }

  if (odo_return == 1) {
    Serial.println("Odometer is ready!");
  } else {
    Serial.print("Odometer couldn't be initialized: ");
    Serial.println(odo_return);
    return 0;
  }

  if (logger_return == 1) {
    Serial.println("Logger is ready!");
    myButton.turn_off_led();
  } else {
    Serial.print("Logger couldn't be initialized: ");
    Serial.println(logger_return);
    return 0;
  }

  return 1;
}
