#include "Robot.h"

Statevars myVars;
Compass myCompass(&myVars);
Gps myGps(&myVars);
LedButton myButton;
Logger myLogger(&myVars);
Mobility myMobility(&myVars);

void setup() {
  Serial.begin(9600);

  // myVars.reset();

  Serial.println(myVars.get_prefix(), HEX);
  Serial.print("Statevars::verify_init(): ");
  Serial.println(myVars.verify_init());

  Serial.print("Compass::verify_init(): ");
  Serial.println(myCompass.verify_init());

  Serial.print("Gps::verify_init(): ");
  Serial.println(myGps.verify_init());

  Serial.print("LedButton::verify_init(): ");
  Serial.println(myButton.verify_init());

  Serial.print("Logger::verify_init(): ");
  Serial.println(myLogger.init_and_verify());

  myMobility.initialize();
  Serial.print("Mobility::verify_init(): ");
  Serial.println(myMobility.verify_init());
}

void loop() {

}
