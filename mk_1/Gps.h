/*
 * file: Gps.h
 * created: 20161009
 * author(s): mr-augustine
 *
 * Describes the class used to initialize and update the GPS sensor.
 */
#ifndef _GPS_H_
#define _GPS_H_

#include <avr/interrupt.h>
#include <avr/io.h>

#include "Statevars.h"

#define GPGGA_START             "$GPGGA"
#define GPGSA_START             "$GPGSA"
#define GPRMC_START             "$GPRMC"
#define GPVTG_START             "$GPVTG"
#define START_LENGTH            6
#define GPS_CHECKSUM_LENGTH     2
#define GPS_INVALID_HEX_CHAR    0xFF
#define GPS_FIELD_BUFF_SZ       8
#define GPS_NO_FIX              '0'
#define GPS_FIX_AVAIL           '1'
#define GPS_DIFF_FIX_AVAIL      '2'
#define GPS_TIME_WIDTH          6
#define GPS_SENTENCE_BUFF_SZ    128
#define GPS_SENTENCE_END        '\n'
#define GPS_SENTENCE_START      '$'
#define LAT_LONG_FIELD_LENGTH   9
#define NUM_GPS_SENTENCE_BUFFS  4

typedef struct {
  uint8_t ready;
  char sentence[GPS_SENTENCE_BUFF_SZ];
} gps_buffer_t;

class Gps {
private:
  uint8_t hexchar_to_dec(char c);
  void initialize_gps_statevars();
  uint8_t parse_gpgga(char * s);
  uint8_t parse_gpgsa(char * s);
  uint8_t parse_gprmc(char * s);
  uint8_t parse_gpvtg(char * s);
  void parse_gps_sentence(char * sentence);
  uint8_t validate_checksum(char * s);

public:
  Gps(Statevars * statevars);
  void initialize(void);
  int8_t verify_init(void);
  void update(void);
};

#endif // #ifndef _GPS_H_
