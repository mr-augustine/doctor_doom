/*
 * file: Statevars.h
 * created: 20161009
 * author(s): mr-augustine
 *
 * This statevars header file defines the data structure that stores the
 * robot's variables. It also defines the status bits which are used to
 * indicate state information about the GPS sensor.
 */
#ifndef _STATEVARS_H_
#define _STATEVARS_H_

#include <stdint.h>

#define DEFAULT_PREFIX        0xDADAFEED
#define DEFAULT_SUFFIX        0xCAFEBABE
#define GPS_SENTENCE_LENGTH   84
#define GPS_DATE_WIDTH        8

#define STATUS_SYS_TIMER_OVERFLOW   (1 << 0)
#define STATUS_MISSION_ACTIVE       (1 << 1)
#define STATUS_GPS_NO_BUFF_AVAIL    (1 << 2)
#define STATUS_GPS_BUFF_OVERFLOW    (1 << 3)
#define STATUS_GPS_UNEXPECT_START   (1 << 4)
#define STATUS_GPS_GPGGA_RCVD       (1 << 5)
#define STATUS_GPS_GPVTG_RCVD       (1 << 6)
#define STATUS_GPS_GPRMC_RCVD       (1 << 7)
#define STATUS_GPS_GPGSA_RCVD       (1 << 8)
#define STATUS_GPS_NO_FIX_AVAIL     (1 << 9)
#define STATUS_GPS_UNEXPECT_VAL     (1 << 10)
#define STATUS_GPS_DATA_NOT_VALID   (1 << 11)
#define STATUS_MAIN_LOOP_LATE       (1 << 12)
#define STATUS_GPS_FIX_AVAIL        (1 << 13)
#define STATUS_NAV_POSITION_KNOWN   (1 << 14)
#define STATUS_COMPASS_ERROR        (1 << 15)

class Statevars {
private:
  uint32_t  prefix;
  uint32_t  status;
  uint32_t  main_loop_counter;
  uint16_t  gps_lat_deg;
  float     gps_lat_ddeg;
  uint16_t  gps_long_deg;
  float     gps_long_ddeg;
  float     gps_hdop;
  float     gps_ground_course_deg;
  float     gps_ground_speed_kt;
  uint8_t   gps_satcount;
  uint16_t  heading_raw;
  float     heading_deg;
  int8_t    pitch_deg;
  int8_t    roll_deg;
  uint32_t  odometer_ticks;
  uint16_t  odometer_timestamp;
  uint8_t   odometer_ticks_are_fwd;
  float     nav_heading_deg;
  float     nav_gps_heading;
  float     nav_latitude;
  float     nav_longitude;
  float     nav_waypt_latitude;
  float     nav_waypt_longitude;
  float     nav_rel_bearing_deg;
  float     nav_distance_to_waypt_m;
  float     nav_speed;
  uint16_t  mobility_motor_pwm;
  uint16_t  mobility_steering_pwm;
  float     control_heading_desired;
  float     control_xtrack_error;
  float     control_xtrack_error_rate;
  float     control_xtrack_error_sum;
  uint16_t  control_steering_pwm;
  uint32_t  suffix;

public:
  void initialize(void);
  int8_t verify_init(void);
  float get_control_heading_desired(void) { return control_heading_desired; }
  void set_control_heading_desired(float chd) { control_heading_desired = chd; }
  uint16_t get_control_steering_pwm(void) { return control_steering_pwm; }
  void set_control_steering_pwm(uint16_t csp) { control_steering_pwm = csp; }
  float get_control_xtrack_error(void) { return control_xtrack_error; }
  void set_control_xtrack_error(float cxe) { control_xtrack_error = cxe; }
  float get_control_xtrack_error_rate(void) { return control_xtrack_error_rate; }
  void set_control_xtrack_error_rate(float cxer) { control_xtrack_error_rate = cxer; }
  float get_control_xtrack_error_sum(void) { return control_xtrack_error_sum; }
  void set_control_xtrack_error_sum(float cxes) { control_xtrack_error_sum = cxes; }
  float get_gps_hdop(void) { return gps_hdop; }
  void set_gps_hdop(float gh) { gps_hdop = gh; }
  float get_gps_ground_course_deg(void) { return gps_ground_course_deg; }
  void set_gps_ground_course_deg(float ggcd) { gps_ground_course_deg = ggcd; }
  float get_gps_ground_speed_kt(void) { return gps_ground_speed_kt; }
  void set_gps_ground_speed_kt(float ggsk) { gps_ground_speed_kt = ggsk; }
  float get_gps_lat_ddeg(void) { return gps_lat_ddeg; }
  void set_gps_lat_ddeg(float gldd) { gps_lat_ddeg = gldd; }
  uint16_t get_gps_lat_deg(void) { return gps_lat_deg; }
  void set_gps_lat_deg(uint16_t gld) { gps_lat_deg = gld; }
  float get_gps_long_ddeg(void) { return gps_long_ddeg; }
  void set_gps_long_ddeg(float gldd) { gps_long_ddeg = gldd; }
  uint16_t get_gps_long_deg(void) { return gps_long_deg; }
  void set_gps_long_deg(uint16_t gld) { gps_long_deg = gld; }
  uint8_t get_gps_satcount(void) { return gps_satcount; }
  void set_gps_satcount(uint8_t gs) { gps_satcount = gs; }
  float get_heading_deg(void) { return heading_deg; }
  void set_heading_deg(float hd) { heading_deg = hd; }
  uint16_t get_heading_raw(void) { return heading_raw; }
  void set_heading_raw(uint16_t hr) { heading_raw = hr; }
  uint32_t get_main_loop_counter(void) { return main_loop_counter; }
  void set_main_loop_counter(uint32_t mlc) { main_loop_counter = mlc; }
  uint16_t get_mobility_motor_pwm(void) { return mobility_motor_pwm; }
  void set_mobility_motor_pwm(uint16_t mmp) { mobility_motor_pwm = mmp; }
  uint16_t get_mobility_steering_pwm(void) { return mobility_steering_pwm; }
  void set_mobility_steering_pwm(uint16_t msp) { mobility_steering_pwm = msp; }
  float get_nav_distance_to_waypt_m(void) { return nav_distance_to_waypt_m; }
  void set_nav_distance_to_waypt_m(float ndtwm) { nav_distance_to_waypt_m = ndtwm; }
  float get_nav_gps_heading(void) { return nav_gps_heading; }
  void set_nav_gps_heading(float ngh) { nav_gps_heading = ngh; }
  float get_nav_heading_deg(void) { return nav_heading_deg; }
  void set_nav_heading_deg(float nhd) { nav_heading_deg = nhd; }
  float get_nav_latitude(void) { return nav_latitude; }
  void set_nav_latitude(float nl) { nav_latitude = nl; }
  float get_nav_longitude(void) { return nav_longitude; }
  void set_nav_longitude(float nl) { nav_longitude = nl; }
  float get_nav_rel_bearing_deg(void) { return nav_rel_bearing_deg; }
  void set_nav_rel_bearing_deg(float nrbd) { nav_rel_bearing_deg = nrbd; }
  float get_nav_speed(void) { return nav_speed; }
  void set_nav_speed(float ns) { nav_speed = ns; }
  float get_nav_waypt_latitude(void) { return nav_waypt_latitude; }
  void set_nav_waypt_latitude(float nwl) { nav_waypt_latitude = nwl; }
  float get_nav_waypt_longitude(void) { return nav_waypt_longitude; }
  void set_nav_waypt_longitude(float nwl) { nav_waypt_longitude = nwl; }
  uint32_t get_odometer_ticks(void) { return odometer_ticks; }
  void set_odometer_ticks(uint32_t ot) { odometer_ticks = ot; }
  uint8_t get_odometer_ticks_are_fwd(void) { return odometer_ticks_are_fwd; }
  void set_odometer_ticks_are_fwd(uint8_t otaf) { odometer_ticks_are_fwd = otaf; }
  uint16_t get_odometer_timestamp(void) { return odometer_timestamp; }
  void set_odometer_timestamp(uint16_t ot) { odometer_timestamp = ot; }
  int8_t get_pitch_deg(void) { return pitch_deg; }
  void set_pitch_deg(int8_t pd) { pitch_deg = pd; }
  uint32_t get_prefix(void) { return prefix; }
  void set_prefix(uint32_t p) { prefix = p; }
  int8_t get_roll_deg(void) { return roll_deg; }
  void set_roll_deg(int8_t rd) { roll_deg = rd; }
  uint32_t get_status(void) { return status; }
  void set_status(uint32_t s) { status = s; }
  uint32_t get_suffix(void) { return suffix; }
  void set_suffix(uint32_t s) { suffix = s; }
};

#endif
