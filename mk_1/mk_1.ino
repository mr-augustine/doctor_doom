#include "Robot.h"

Statevars myVars;
Mobility myMobility(&myVars);

void setup() {
  Serial.begin(9600);

  myVars.reset();

  Serial.println(myVars.get_prefix(), HEX);
}

void loop() {

}
