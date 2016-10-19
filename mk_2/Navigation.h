#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <math.h>

#include "Compass.h"
#include "Gps.h"
#include "Odometer.h"
#include "Statevars.h"

#define DEG_TO_RAD(degrees)         (degrees * M_PI / 180.0)
#define RAD_TO_DEG(radians)         (radians * 180.0 / M_PI)
#define EARTH_RADIUS_M              6371393.0
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
// And stolen from NGDC: http://www.ngdc.noaa.gov/geomag-web/
#define MAGNETIC_DECLINATION        4.0  // For central Texas
#define METERS_PER_SECOND_PER_KNOT  0.514444
#define TICKS_PER_METER             7.6

class Navigation {
private:
  Statevars * vars;
  Compass * compass;
  Gps * gps;
  Odometer * odometer;

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

  float seconds_per_tick;

  float calc_dist_to_waypoint(float start_lat, float start_long, float end_lat, float end_long);
  float calc_mid_angle(float heading_1, float heading_2);
  float calc_nav_heading(void);
  void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading);
  float calc_speed_from_distance(float distance_m);
  float calc_speed_from_ticks(uint32_t ticks);
  float calc_true_bearing(float start_lat, float start_long, float dest_lat, float dest_long);
  void get_next_waypoint(void);

public:
  static float calc_relative_bearing(float desired_bearing, float current_heading);

  Navigation(const Statevars * s, const Compass * c, const Gps * g, const Odometer * o, float spt);
  void initialize(void);
  int8_t verify_init(void);
  void update(void);
};

#endif
