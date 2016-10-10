/*
 * file: Robot.h
 * created: 20161009
 * author(s): mr-augustine
 *
 * The robot header file describes the class that maintains all of the robot's
 * higher-order functions.
 */
#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "Gps.h"
#include "LedButton.h"
#include "Mobility.h"
#include "Statevars.h"

#define ROBOT_NAME                  ("doom")

#define DEG_TO_RAD(degrees)         (degrees * M_PI / 180.0)
#define RAD_TO_DEG(radians)         (radians * 180.0 / M_PI)
#define EARTH_RADIUS_M              6371393.0
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
// And stolen from NGDC: http://www.ngdc.noaa.gov/geomag-web/
#define MAGNETIC_DECLINATION        4.0  // For central Texas
#define METERS_PER_SECOND_PER_KNOT  0.514444
//TODO make this a const that's calculated during init; use a big-enough datatype
#define MICROS_PER_TICK             4.0
//TODO make this a const that's calcualted during init
#define SECONDS_PER_TICK            (MICROS_PER_TICK / 1000000.0)
#define TICKS_PER_METER             7.6

//TODO make this a const that's calculated during init
#define SECONDS_PER_LOOP            (PWM_PERIOD_MS / 1000.0)

#define TARGET_HEADING              270.0

#define K_PROP                      10 // proportional gain
#define K_RATE                      0 // derivative gain
#define K_INTEGRAL                  0 // integral gain

class Robot {
private:
  Statevars * vars;
  //Compass * compass;
  Gps * gps;
  LedButton * button;
  //Odometer * odometer;

  float current_lat;
  float current_long;
  float waypoint_lat;
  float waypoint_long;
  float nav_heading_deg;
  float rel_bearing_deg;
  float distance_to_waypoint_m;
  float current_speed; // in meters per second
  float waypt_true_bearing;
  float gps_lat_most_recent;
  float gps_long_most_recent;
  float gps_hdg_most_recent;
  float gps_speed_most_recent; // in meters per second
  uint32_t prev_tick_count;
  uint8_t got_first_coord = 0;
  float xtrack_error;
  float xtrack_error_prev;
  float xtrack_error_rate;
  float xtrack_error_sum;
  float steer_control = 1500;

  float calc_dist_to_waypoint(float start_lat, float start_long, float end_lat, float end_long);
  float calc_mid_angle(float heading_1, float heading_2);
  float calc_nav_heading(void);
  void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading);
  float calc_relative_bearing(float desired_bearing, float current_heading);
  float calc_speed(float distance_m);
  float calc_speed_mps(uint32_t ticks);
  float calc_true_bearing(float start_lat, float start_long, float dest_lat, float dest_long);
  void get_next_waypoint(void);
  void update_all_nav(void);
  void update_xtrack_error(void);
  void update_xtrack_error_rate(void);
  void update_xtrack_error_sum(void);

public:
  Robot(const Statevars * s, const Gps * g, const LedButton * l);
  void update_all_inputs(void);
  void update_nav_control_values(void);
};

#endif // #ifndef _ROBOT_H_
