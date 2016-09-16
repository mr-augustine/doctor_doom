#ifndef _STATEVARS_H_
#define _STATEVARS_H_

#include <stdint.h>

#define GPS_SENTENCE_LENGTH   84
#define GPS_DATE_WIDTH        8

#define STATUS_SYS_TIMER_OVERFLOW (1 << 0)
#define STATUS_MISSION_ACTIVE     (1 << 1)
#define STATUS_GPS_NO_BUFF_AVAIL  (1 << 2)
#define STATUS_GPS_BUFF_OVERFLOW  (1 << 3)
#define STATUS_GPS_UNEXPECT_START (1 << 4)
#define STATUS_GPS_GPGGA_RCVD     (1 << 5)
#define STATUS_GPS_GPVTG_RCVD     (1 << 6)
#define STATUS_GPS_GPRMC_RCVD     (1 << 7)
#define STATUS_GPS_GPGSA_RCVD     (1 << 8)
#define STATUS_GPS_NO_FIX_AVAIL   (1 << 9)
#define STATUS_GPS_UNEXPECT_VAL   (1 << 10)
#define STATUS_GPS_DATA_NOT_VALID (1 << 11)
#define STATUS_MAIN_LOOP_LATE     (1 << 12)
#define STATUS_GPS_FIX_AVAIL      (1 << 13)
#define STATUS_NAV_POSITION_KNOWN (1 << 14)

typedef struct {
    uint32_t  prefix;
    uint32_t  status;
    uint32_t  main_loop_counter;
    char      gps_sentence0[GPS_SENTENCE_LENGTH];
    char      gps_sentence1[GPS_SENTENCE_LENGTH];
    char      gps_sentence2[GPS_SENTENCE_LENGTH];
    char      gps_sentence3[GPS_SENTENCE_LENGTH];
    float     gps_latitude;
    float     gps_longitude;
    uint16_t  gps_lat_deg;
    float     gps_lat_ddeg;
    uint16_t  gps_long_deg;
    float     gps_long_ddeg;
    float     gps_hdop;
    float     gps_pdop;
    float     gps_vdop;
    float     gps_msl_altitude_m;
    float     gps_true_hdg_deg;
    float     gps_ground_course_deg;
    float     gps_speed_kmph;
    float     gps_ground_speed_kt;
    float     gps_speed_kt;
    uint8_t   gps_hours;
    uint8_t   gps_minutes;
    float     gps_seconds;
    char      gps_date[GPS_DATE_WIDTH];
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
    float     control_steering_pwm;
    uint32_t suffix;
} statevars_t;

extern statevars_t statevars;

#endif // #ifndef _STATEVARS_H_
