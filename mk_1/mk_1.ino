#include "Robot.h"

Statevars myVars;
Mobility myMobility(&myVars);
Compass myCompass(&myVars);
Gps myGps(&myVars);

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
}

void loop() {

}
