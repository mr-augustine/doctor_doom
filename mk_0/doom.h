#ifndef _DOOM_H_
#define _DOOM_H_

#include <math.h>

#include "cmps10.h"
#include "gps.h"
#include "statevars.h"
#include "ledbutton.h"
#include "mobility.h"
#include "odometer.h"
#include "sdcard.h"

#define DEG_TO_RAD(degrees) (degrees * M_PI / 180.0)
#define RAD_TO_DEG(radians) (radians * 180.0 / M_PI)
#define EARTH_RADIUS_M 6371393.0
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
// And stolen from NGDC: http://www.ngdc.noaa.gov/geomag-web/
#define MAGNETIC_DECLINATION 8.52  // For Boulder, Colorado
#define METERS_PER_SECOND_PER_KNOT 0.514444
#define MICROS_PER_TICK 4.0
#define SECONDS_PER_TICK 0.000004
#define TICKS_PER_METER 7.6

#define SECONDS_PER_LOOP 0.025

#define TARGET_HEADING 270.0

#define K_PROP 10 // proportional gain
#define K_RATE 0 // derivative gain
#define K_INTEGRAL 0 // integral gain

static float current_lat;
static float current_long;
static float waypoint_lat;
static float waypoint_long;
static float nav_heading_deg;
static float rel_bearing_deg;
static float distance_to_waypoint_m;
static float current_speed; // in meters per second
static float waypt_true_bearing;

static float gps_lat_most_recent;
static float gps_long_most_recent;
static float gps_hdg_most_recent;
static float gps_speed_most_recent; // in meters per second
static uint32_t prev_tick_count;

static uint8_t got_first_coord = 0;

static float xtrack_error;
static float xtrack_error_prev;
static float xtrack_error_rate;
static float xtrack_error_sum;

static float steer_control = 1500;

static float calc_dist_to_waypoint(float start_lat, float start_long, float end_lat, float end_long);
static float calc_mid_angle(float heading_1, float heading_2);
static float calc_nav_heading(void);
static void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading);
static float calc_relative_bearing(float desired_bearing, float current_heading);
static float calc_speed(float distance_m);
static float calc_speed_mps(uint32_t ticks);
static float calc_true_bearing(float start_lat, float start_long, float dest_lat, float dest_long);
static void get_next_waypoint(void);
static void update_xtrack_error(void);
static void update_xtrack_error_rate(void);
static void update_xtrack_error_sum(void);

void update_all_inputs(void);
void update_nav_control_values(void);

// ******** FUNCTION DEFINITIONS BELOW ******** //

// Returns the distance (in meters) to the current waypoint
static float calc_dist_to_waypoint(float lat_1, float long_1, float lat_2, float long_2) {
  float lat_1_rad = DEG_TO_RAD(lat_1);
  float long_1_rad = DEG_TO_RAD(long_1);
  float lat_2_rad = DEG_TO_RAD(lat_2);
  float long_2_rad = DEG_TO_RAD(long_2);

  float diff_lat = lat_2_rad - lat_1_rad;
  float diff_long = long_2_rad - long_1_rad;

  float a = ( pow(sin(diff_lat / 2), 2) ) +
    cos(lat_1_rad) *
    cos(lat_2_rad) *
    ( pow(sin(diff_long / 2), 2) );

  float c = 2 * asin(sqrt(a));

  float distance_m = EARTH_RADIUS_M * c;

  return distance_m;
}

// Returns the angle that is halfway between the specified headings
static float calc_mid_angle(float heading_1, float heading_2) {
  // Ensure that heading_2 stores the larger heading
  if (heading_1 > heading_2) {
    float temp = heading_1;
    heading_1 = heading_2;
    heading_2 = temp;
  }

  if (heading_2 - heading_1 > 180.0) {
    heading_2 -= 360.0;
  }

  float mid_angle = (heading_2 + heading_1) / 2.0;

  if (mid_angle < 0.0) {
    mid_angle += 360.0;
  }

  return mid_angle;
}

static float calc_nav_heading(void) {
  float norm_mag_hdg = statevars.heading_deg + MAGNETIC_DECLINATION;

  if (norm_mag_hdg > 360.0) {
    norm_mag_hdg -= 360.0;
  }

  // Here we're calculating the navigation heading as the mid-angle between
  // the compass heading and the GPS heading because experimental data seemed
  // to produce good results when we did this.
  float nav_heading = calc_mid_angle(norm_mag_hdg, gps_hdg_most_recent);

  statevars.nav_heading_deg = nav_heading;

  // If you just want a compass-based heading, use this
  // return norm_mag_hdg;

  return nav_heading;
}

// Calculates a new position based on the current heading and distance
// traveled from the previous position
static void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading) {
  float lat_rad = DEG_TO_RAD(ref_lat);
  float long_rad = DEG_TO_RAD(ref_long);
  float heading_rad = DEG_TO_RAD(heading);

  float est_lat = asin( sin(lat_rad) *
    cos(distance / EARTH_RADIUS_M) +
    cos(lat_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(heading_rad) );

  float est_long = long_rad +
    atan2( sin(heading_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(lat_rad),
    cos(distance / EARTH_RADIUS_M) -
    sin(lat_rad) *
    sin(est_lat) );

  *new_lat = RAD_TO_DEG(est_lat);
  *new_long = RAD_TO_DEG(est_long);

  return;
}

// Calculates the relative bearing in degrees (i.e., the angle between the current
// heading and the waypoint bearing); a negative value means the destination
// is towards the left, and vice versa
// "I'd have to change my heading by this much to point to the waypoint"
static float calc_relative_bearing(float desired_bearing, float current_heading) {
  float diff = desired_bearing - current_heading;

  // We want the range of bearings to be between -180..+180; so a result of
  // -225 (225 degrees to the left of where I'm pointing) will become +135
  // (135 degrees to the right of where I'm pointing)
  if (diff > 180.0) {
    return (diff - 360.0);
  } else if (diff < -180.0) {
    return (diff + 360.0);
  }

  return diff;
}

// Calculate the robot's current speed based on how many odometer ticks were
// measured; result is in meters per second
static float calc_speed_mps(uint32_t ticks) {
  if (ticks == 0) {
    return 0.0;
  }

  float distance_m = ticks / TICKS_PER_METER;
  float elapsed_time_s = statevars.odometer_timestamp * SECONDS_PER_TICK;

  float speed_meters_per_sec = 0.0;

  if (elapsed_time_s > 0.0) {
    speed_meters_per_sec = distance_m / elapsed_time_s;
  }

  return speed_meters_per_sec;
}

// Calculates the robot's current speed; result in meters per second
static float calc_speed(float distance_m) {
  float elapsed_time_s = statevars.odometer_timestamp * MICROS_PER_TICK / 1000000.0;

  float speed_meters_per_sec = 0.0;

  if (elapsed_time_s > 0.0) {
    speed_meters_per_sec = distance_m / elapsed_time_s;
  }

  return speed_meters_per_sec;
}

// Calculates the true bearing between two gps coordinates in degrees
// "I'd have to change my heading to this value to point to that coordinate"
static float calc_true_bearing(float start_lat, float start_long, float dest_lat, float dest_long) {
  float start_lat_rad = DEG_TO_RAD(start_lat);
  float start_long_rad = DEG_TO_RAD(start_long);
  float dest_lat_rad = DEG_TO_RAD(dest_lat);
  float dest_long_rad = DEG_TO_RAD(dest_long);

  float y = sin(dest_long_rad - start_long_rad) *
    cos(dest_lat_rad);
  float x = cos(start_lat_rad) *
    sin(dest_lat_rad) -
    sin(start_lat_rad) *
    cos(dest_lat_rad) *
    cos(dest_long_rad - start_long_rad);

  float bearing_rad = atan2(y, x);
  float bearing_deg = RAD_TO_DEG(bearing_rad);

  // Shift the values from the range [-180,180] to [0,360)
  if (bearing_deg < 0.0) {
    return bearing_deg + 360.0;
  }

  return bearing_deg;
}

// Gets the next waypoint
// For this demo, we're using the decimal degrees of the first GPS coordinate
// we received
static void get_next_waypoint(void) {
  if (got_first_coord == 1) {
    return;
  }

  if (statevars.status & STATUS_GPS_FIX_AVAIL) {
    // Ensure we aren't getting the default lat/long
    //TODO We're assuming that the waypoint and current position don't cross
    //a whole degree boundary!!! Fix this!
    if (statevars.gps_latitude != 0.0 && statevars.gps_longitude != 0.0) {
      waypoint_lat = statevars.gps_lat_ddeg;
      waypoint_long = statevars.gps_long_ddeg;

      got_first_coord = 1;
    }
  }

  return;
}

static void update_all_nav(void) {
  get_next_waypoint();

  // Check if a new GPS coordinate was received and update the position
  if (statevars.status & STATUS_GPS_FIX_AVAIL) {
    // Calculate a new gps-based heading using the previous coord (current)
    // and the newest coord (statevars)
    gps_hdg_most_recent = calc_true_bearing(gps_lat_most_recent,
                                            gps_long_most_recent,
                                            statevars.gps_lat_ddeg,
                                            statevars.gps_long_ddeg);

    gps_lat_most_recent = statevars.gps_lat_ddeg;
    gps_long_most_recent = statevars.gps_long_ddeg;
    current_lat = statevars.gps_lat_ddeg;
    current_long = statevars.gps_long_ddeg;
  }

  // Check if a new GPS heading and speed were received and update
  if (statevars.status & STATUS_GPS_GPRMC_RCVD) {
    // Not using the gps heading because sometimes it's horrendous. Instead,
    // we'll calculate our own using calc_true_bearing()
    // gps_hdg_most_recent = statevars.gps_ground_course_deg;
    gps_speed_most_recent = statevars.gps_ground_speed_kt * METERS_PER_SECOND_PER_KNOT;
  }

  // Calculate the number of ticks that occurred during the current iteration
  // Since the tick count is cumulative, the new tick count will always be
  // greater-than or equal to the previous tick count
  uint32_t new_tick_count = statevars.odometer_ticks;
  uint32_t tick_diff = new_tick_count - prev_tick_count;

  current_speed = calc_speed_mps(tick_diff);

  // TODO I know; we're doing another tick_diff / TICKS_PER_METER calculation.
  // But we'll optimize this later
  float distance_since_prev_iter_m = tick_diff / TICKS_PER_METER;

  // We're using calc_speed_mps() instead to allow integer-based distance diff eval
  // current_speed = calc_speed(distance_since_prev_iter_m);

  // Advance the tick count now that we're done with the previous value
  prev_tick_count = new_tick_count;

  if (current_speed == 0.0) {
    current_speed = gps_speed_most_recent;
  }

  nav_heading_deg = calc_nav_heading();

  float old_lat = current_lat;
  float old_long = current_long;
  calc_position(&current_lat, &current_long, old_lat, old_long, distance_since_prev_iter_m, nav_heading_deg);

  waypt_true_bearing = calc_true_bearing(current_lat, current_long, waypoint_lat, waypoint_long);
  rel_bearing_deg = calc_relative_bearing(waypt_true_bearing, nav_heading_deg);

  distance_to_waypoint_m = calc_dist_to_waypoint(current_lat, current_long, waypoint_lat, waypoint_long);

  statevars.nav_heading_deg = nav_heading_deg;
  statevars.nav_gps_heading = gps_hdg_most_recent;
  statevars.nav_latitude = current_lat;
  statevars.nav_longitude = current_long;
  statevars.nav_waypt_latitude = waypoint_lat;
  statevars.nav_waypt_longitude = waypoint_long;
  statevars.nav_rel_bearing_deg = rel_bearing_deg;
  statevars.nav_distance_to_waypt_m = distance_to_waypoint_m;
  statevars.nav_speed = current_speed;

  return;
}

static void update_xtrack_error(void) {
  // Error = Reference Value - Measured value
  xtrack_error_prev = xtrack_error;

  xtrack_error = calc_relative_bearing(TARGET_HEADING, nav_heading_deg);

  // TODO Change this assignment when you start navigating to waypoints.
  // Normally the desired heading will be the waypt_true_bearing unless we set
  // a new bearing if an obstacle is detected
  statevars.control_heading_desired = TARGET_HEADING;
  // statevars.control_heading_desired = waypt_true_bearing;

  statevars.control_xtrack_error = xtrack_error;
  // TODO Normally the cross track error will be the relative bearing (to the
  // waypoint) unless we decide to set a new bearing if an obstacle is detected
  // statevars.control_xtrack_error = rel_bearing_deg;

  return;
}

static void update_xtrack_error_rate(void) {
  // Rate = (Error - Error_Previous) / Computation Interval
  xtrack_error_rate = (xtrack_error - xtrack_error_prev) / SECONDS_PER_LOOP;

  statevars.control_xtrack_error_rate = xtrack_error_rate;

  return;
}

static void update_xtrack_error_sum(void) {
  // TODO Be sure to reset the cross track error sum  to zero when the
  // cross track error becomes zero (or very close to it)

  // Rate Sum = Rate Sum + Error * Computation Interval
  xtrack_error_sum = xtrack_error_sum + xtrack_error * SECONDS_PER_LOOP;

  statevars.control_xtrack_error_sum = xtrack_error_sum;

  return;
}

void update_all_inputs(void) {
  button_update();

  // TODO Enable these
  cmps10_update_all();
  gps_update();
  odometer_update();

  update_all_nav();

  return;
}

void update_nav_control_values(void) {
  update_xtrack_error();
  update_xtrack_error_rate();
  update_xtrack_error_sum();

  steer_control = (K_PROP * xtrack_error) +
                  (K_RATE * xtrack_error_rate) +
                  (K_INTEGRAL * xtrack_error_sum);

  // Limit validation for steer_control will be handled by the mobility library.
  // This way we don't command the robot to turn beyond the full left/right
  // steering angles

  // If the xtrack_error is NEGATIVE, then the robot is towards the RIGHT of
  // where it needs to be; If the xtrack error is POSITIVE, then the robot is
  // towards the LEFT of where it needs to be. So, if I'm towards the right of
  // the target heading, I need to turn left. To turn left, you increase the
  // steering PWM value. This is why we have a subtraction in the line below.
  statevars.control_steering_pwm = (uint16_t)(TURN_NEUTRAL - steer_control);
  //statevars.control_steering_pwm = (uint16_t)(statevars.mobility_steering_pwm - steer_control);

  return;
}

#endif // _DOOM_H_
