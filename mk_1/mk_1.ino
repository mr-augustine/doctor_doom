#include "Robot.h"

Statevars myVars;
Compass myCompass(&myVars);
Gps myGps(&myVars);
LedButton myButton;
Logger myLogger(&myVars);
Mobility myMobility(&myVars);

void setup() {
  Serial.begin(9600);

  init_all_subsystems();

  verify_all_subsystems();
}

void loop() {

}

void init_all_subsystems(void) {
  cli();

  myVars.initialize();
  myCompass.initialize();
  myGps.initialize();
  myButton.initialize();
  myMobility.initialize();

  sei();

  // The logger needs interrupts enabled so that it can initialize
  myLogger.initialize();

  return;
}

uint8_t verify_all_subsystems(void) {
  int8_t svars_return = myVars.verify_init();
  int8_t cmps_return = myCompass.verify_init();
  int8_t gps_return = myGps.verify_init();
  int8_t button_return = myButton.verify_init();
  int8_t logger_return = myLogger.verify_init();
  int8_t mobility_return = myMobility.verify_init();

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
