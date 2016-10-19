/*
 * file: Robot.cpp
 * created: 20161009
 * author(s): mr-augustine
 */
#include "Robot.h"

Robot::Robot( const Statevars * s,
              const Compass * cmps,
              const Control * ctrl,
              const Gps * g,
              const LedButton * l,
              const Odometer * o,
              const Navigation * n) {
  vars = s;
  compass = cmps;
  control = ctrl;
  gps = g;
  button = l;
  odometer = o;
  navigation = n;
}

void Robot::update(void) {
  button->update();
  compass->update();
  gps->update();
  odometer->update();
  navigation->update();

  // Control update must be performed last
  control->update();

  return;
}
