/*
 * file: Robot.h
 * created: 20161017
 * author(s): mr-augustine
 *
 * The robot header file describes the class that maintains all of the robot's
 * higher-order functions.
 */
#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "Compass.h"
#include "Control.h"
#include "Gps.h"
#include "LedButton.h"
#include "Logger.h"
#include "Mobility.h"
#include "Odometer.h"
#include "Navigation.h"
#include "Statevars.h"

#define ROBOT_NAME                  ("doom")

#define MAIN_LOOP_FREQ_HZ           40

//TODO make this a const that's calculated during init
#define MAIN_LOOP_PERIOD_SEC        (1.0 / MAIN_LOOP_FREQ_HZ)


#define MAIN_LOOP_TIMER_PRESCALER   64
#define CLOCK_SPEED_HZ              16000000.0

class Robot {
private:
  Statevars * vars;
  Compass * compass;
  Control * control;
  Gps * gps;
  LedButton * button;
  Odometer * odometer;
  Navigation * navigation;

public:
  Robot(const Statevars * s, const Compass * cmps, const Control * ctrl, const Gps * g, const LedButton * l, const Odometer * o, const Navigation * n);
  void update(void);
};

#endif // #ifndef _ROBOT_H_
