/*
 * file: Gps.h
 * created: 20161009
 * author(s): mr-augustine
 */
#include <string.h>
#include <stdlib.h>

#include "Gps.h"

static int8_t buffer_index;
static uint8_t sentence_index;
static gps_buffer_t gps_buffers[NUM_GPS_SENTENCE_BUFFS];

static volatile uint8_t gps_no_buff_avail = 0;
static volatile uint8_t gps_buff_overflow = 0;
static volatile uint8_t gps_unexpected_start = 0;

/* Interrupt Service Routine that triggers whenever a new character
 * is received from the GPS sensor. Adds the new char to a buffer
 * such that all chars from the same sentence are saved to the same
 * buffer. Each new sentence is saved to the first available buffer.
 */
ISR (USART2_RX_vect) {
  char new_char = UDR2;

  // If a gps buffer hasn't been identified to be filled,
  // look for the first available buffer to start writing to
  if (buffer_index == -1) {
    uint8_t i;

    for (i = 0; i < NUM_GPS_SENTENCE_BUFFS; i++) {
      if (gps_buffers[i].ready == 0) {
        buffer_index = i;
        sentence_index = 0;
        break;
      }
    }

    // No available buffers were found
    if (i == NUM_GPS_SENTENCE_BUFFS) {
      gps_no_buff_avail = 1;
      return;
    }
  }

  // If we received a sentence_start character while in the middle
  // of populating a buffer, mark this as unexpected and prepare
  // to overwrite the current buffer starting at the beginning
  if (new_char == GPS_SENTENCE_START && sentence_index != 0) {
    gps_unexpected_start = 1;
    buffer_index = 0;
  }

  // If we received a data character or and unexpected start or
  // a legitimate start, then add the character to the buffer
  if (new_char != GPS_SENTENCE_END) {
    gps_buffers[buffer_index].sentence[sentence_index] = new_char;
    sentence_index = sentence_index + 1;

    // Verify that the buffer has enough room for the carriage return,
    // newline, and null chars. If there isn't enough room, rollback the index.
    if (sentence_index == GPS_SENTENCE_BUFF_SZ - 2) {
      buffer_index = buffer_index - 1;
      gps_buff_overflow = 1;
    }

    return;
  }

  // We received a newline character, so terminate the current sentence buffer
  gps_buffers[buffer_index].sentence[sentence_index++] = new_char;
  gps_buffers[buffer_index].sentence[sentence_index] = '\0';
  gps_buffers[buffer_index].ready = 1;

  buffer_index = -1;
}

Gps::Gps(const Statevars * s) {
  vars = s;
}

void Gps::initialize(void) {
  buffer_index = -1;
  sentence_index = 0;

  // Unset the flags, double speed, and comm mode bits
  //UCSR0A = 0;

  // TODO Consider defining these registers in Pins.h
  // Enable receive interrupt and receiving
  UCSR2B = 0;
  UCSR2B = (1 << RXCIE2) | (1 << RXEN2);

  // Enable 8-bit character size
  // Asynchronous USART, no parity, 1 stop bit already set (default)
  UCSR2C = 0;
  UCSR2C = (1 << UCSZ11) | (1 << UCSZ10);

  // Set baud rate to 9600; 115200 doesn't work
  // See Table 22-12 in the Atmel 2560 specs
  UBRR2H = 0;
  UBRR2L = 103; //baud = 9600

  return;
}

int8_t Gps::verify_init(void) {
  // Verify receive interrupt and receiving are enabled
  if ( UCSR2B != ((1 << RXCIE2) | (1 << RXEN2)) ) {
    return 0;
  }

  // Verify that asynchronous USART, no parity, 1 stop bid
  if ( UCSR2C != ((1 << UCSZ11) | (1 << UCSZ10)) ) {
    return -1;
  }

  // Verify baud rate is 9600
  if (UBRR2H != 0 || UBRR2L != 103) {
    return -2;
  }

  return 1;
}

void Gps::update(void) {
  initialize_gps_statevars();

  if (gps_no_buff_avail == 1) {
    vars->set_status(vars->get_status() | STATUS_GPS_NO_BUFF_AVAIL);
    gps_no_buff_avail = 0;
  }

  if (gps_buff_overflow == 1) {
    vars->set_status(vars->get_status() | STATUS_GPS_BUFF_OVERFLOW);
    gps_buff_overflow = 0;
  }

  if (gps_unexpected_start == 1) {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_START);
    gps_unexpected_start = 0;
  }

  for (uint8_t i = 0; i < NUM_GPS_SENTENCE_BUFFS; i++) {
    if (gps_buffers[i].ready == 1) {
      char * sentence_ptr = gps_buffers[i].sentence;

      parse_gps_sentence(sentence_ptr);

      memset(sentence_ptr, '\0', GPS_SENTENCE_BUFF_SZ);
      gps_buffers[i].ready = 0;
    }
  }

  return;
}

uint8_t Gps::hexchar_to_dec(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }

  return GPS_INVALID_HEX_CHAR;
}

// Resets all GPS-related statevars to zero
void Gps::initialize_gps_statevars(void) {
  vars->set_gps_lat_deg(0);
  vars->set_gps_lat_ddeg(0.0);
  vars->set_gps_long_deg(0);
  vars->set_gps_long_ddeg(0.0);
  vars->set_gps_hdop(0.0);
  vars->set_gps_ground_course_deg(0.0);
  vars->set_gps_ground_speed_kt(0.0);
  vars->set_gps_satcount(0);

  return;
}

/* Parses the specified GPGGA sentence and saves the values of interest
 * to the statevars variable.
 *
 * Note: The current implementation is destructive because it uses strtok()
 * to tokenize the sentence. The ',' delimeters will be overwritten with
 * null chars.
 *
 * Returns 0 if successful; 1 otherwise
 */
uint8_t Gps::parse_gpgga(char * s) {
  char field_buf[GPS_FIELD_BUFF_SZ];
  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);

  // $GPGGA header - ignore
  s = strtok(s, ",");

  // UTC Time - hhmmss.sss
  s = strtok(NULL, ",");
  // strncpy(field_buf, s, 2);
  // vars->set_gps_hours(atoi(field_buf));

  // memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  // strncpy(field_buf, s+2, 2);
  // vars->set_gps_minutes(atoi(field_buf));

  // memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  // strncpy(field_buf, s+4, 6);
  // vars->set_gps_seconds(atof(field_buf));

  // Latitude - ddmm.mmmm
  s = strtok(NULL, ",");
  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  strncpy(field_buf, s, 2);
  int16_t lat_degrees = atoi(field_buf);

  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  strncpy(field_buf, s+2, 7);
  float lat_minutes = atof(field_buf);

  // Latitude Hemisphere
  s = strtok(NULL, ",");
  uint8_t lat_is_south;
  if (*s == 'N') {
    lat_is_south = 0;
  } else if (*s == 'S') {
    lat_is_south = 1;
  } else {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }

  // Longitude - dddmm.mmmm
  s = strtok(NULL, ",");
  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  strncpy(field_buf, s, 3);
  int16_t long_degrees = atoi(field_buf);

  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);
  strncpy(field_buf, s+3, 7);
  float long_minutes = atof(field_buf);

  // Longitude Hemisphere
  s = strtok(NULL, ",");
  uint8_t long_is_west;
  if (*s == 'W') {
    long_is_west = 1;
  } else if (*s == 'E') {
    long_is_west = 0;
  } else {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }

  float lat_decimal_degrees = lat_minutes / 60.0;
  float latitude = lat_degrees + lat_decimal_degrees;
  if (lat_is_south) {
    latitude = -latitude;
    lat_degrees = -lat_degrees;
    lat_decimal_degrees = -lat_decimal_degrees;
  }

  float long_decimal_degrees = long_minutes / 60.0;
  float longitude = long_degrees + long_decimal_degrees;
  if (long_is_west) {
    longitude = -longitude;
    long_degrees = -long_degrees;
    long_decimal_degrees = -long_decimal_degrees;
  }

  // vars->set_gps_latitude(latitude);
  // vars->set_gps_longitude(longitude);
  vars->set_gps_lat_deg(lat_degrees);
  vars->set_gps_lat_ddeg(lat_decimal_degrees);
  vars->set_gps_long_deg(long_degrees);
  vars->set_gps_long_ddeg(long_decimal_degrees);

  // Position (Fix) Indicator
  s = strtok(NULL, ",");
  // For some reason, the GPS sensor uses the differential_gps_fix code (2)
  // for the fix indicator instead of gps_fix code (1). Take them either way.
  if (*s == GPS_DIFF_FIX_AVAIL || *s == GPS_FIX_AVAIL) {
    vars->set_status(vars->get_status() | STATUS_GPS_FIX_AVAIL);
  }
  // If there is no fix, set an error flag
  else if (*s == GPS_NO_FIX) {
    vars->set_status(vars->get_status() | STATUS_GPS_NO_FIX_AVAIL);
  }
  // If we get some other code, error out
  else {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }

  // Satellite Count
  s = strtok(NULL, ",");
  vars->set_gps_satcount(atoi(s));

  // Horizontal Dilution of Precision (HDOP)
  s = strtok(NULL, ",");
  vars->set_gps_hdop(atof(s));

  // Mean Sea Level Altitude
  s = strtok(NULL, ",");
  // vars->set_gps_msl_altitude_m(atof(s));

  return 0;
}

// pdop, vdop
uint8_t Gps::parse_gpgsa(char * s) {
  // $GPGSA header - ignore
  s = strtok(s, ",");

  // Mode 1 - ignore
  s = strtok(NULL, ",");

  // Mode 2 - ignore
  s = strtok(NULL, ",");

  // Satellite Used (12 total) - ignore all
  // Note: empty fields (e.g., ",,,,") will be skipped by strtok
  // So we'll look for the first occurrence of a field with a decimal point.
  // This first occurrence will be the PDOP field.
  uint8_t i;
  for (i = 0; i < 12; i++) {
    s = strtok(NULL, ",");

    // We found the PDOP field
    if (*(s + 1) == '.') {
      break;
    }
  }

  // Position Dilution of Precision (PDOP)
  // We only advance the cursor if all satellite fields contained values.
  // If there is at least one unused satellite field, then the cursor would
  // already be pointing to the PDOP field.
  if (i == 12) {
    s = strtok(NULL, ",");
  }
  // vars->set_gps_pdop(atof(s));

  // HDOP - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Vertical Dilution of Precision (VDOP)
  s = strtok(NULL, ",");
  // vars->set_gps_vdop(atof(s));

  return 0;
}

// speed over ground, course over ground, date, magnetic variation
uint8_t Gps::parse_gprmc(char * s) {
  // $GPRMC header - ignore
  s = strtok(s, ",");

  // UTC Time - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Status
  s = strtok(NULL, ",");
  // 'A' == data valid; anything else is an error
  // FYI: 'V' == data not valid
  if (*s != 'A') {
    vars->set_status(vars->get_status() | STATUS_GPS_DATA_NOT_VALID);
    return 1;
  }

  // Latitude - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Latitude Hemisphere - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Longitude - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Longitude Hemisphere - ignore (we get this from $GPGGA)
  s = strtok(NULL, ",");

  // Speed over ground
  s = strtok(NULL, ",");
  vars->set_gps_ground_speed_kt(atof(s));

  // True course over ground
  s = strtok(NULL, ",");
  vars->set_gps_ground_course_deg(atof(s));

  // Date - ddmmyy
  s = strtok(NULL, ",");
  // strncpy(vars->get_gps_date(), s, GPS_FIELD_BUFF_SZ);

  // Ignoring Magnetic variation - ignoring; this won't exist because we
  // haven't configured the GPS sensor to produce this value

  // Ignoring Magnetic variation direction

  // Ignoring Mode field

  return 0;
}

// true course in deg, speed in knots, speed in km/hr
uint8_t Gps::parse_gpvtg(char * s) {
  char field_buf[GPS_FIELD_BUFF_SZ];
  memset(field_buf, '\0', GPS_FIELD_BUFF_SZ);

  // $GPVTG header - ignore
  s = strtok(s, ",");

  // Course - True heading
  // only write the course value to statevars if the reference field that follows
  // is valid
  s = strtok(NULL, ",");
  float true_hdg_deg = atof(s);

  // Course reference
  s = strtok(NULL, ",");
  if (*s != 'T') {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }
  // vars->set_gps_true_hdg_deg(true_hdg_deg);

  // Course - Magnetic heading - won't exist for us since we haven't configured
  // the gps sensor to provide this. This should point to the next field ('M').
  s = strtok(NULL, ",");

  // Course reference - ignore (belongs with course magnetic heading)
  // Note: We don't need to advance the cursor since the previous field doesn't
  // exist
  //s = strtok(NULL, ",");

  // Horizontal speed in knots
  // only write the speed value to statevars if the reference field that follows
  // is valid
  s = strtok(NULL, ",");
  float speed_knots = atof(s);

  // Speed reference
  s = strtok(NULL, ",");
  if (*s != 'N') {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }
  // vars->set_gps_speed_kt(speed_knots);

  // Horizontal speed in kmph
  // only write the speed value to statevars if the reference field that follows
  // is valid
  s = strtok(NULL, ",");
  float speed_kmph = atof(s);

  // Speed reference
  s = strtok(NULL, ",");
  if (*s != 'K') {
    vars->set_status(vars->get_status() | STATUS_GPS_UNEXPECT_VAL);
    return 1;
  }
  // vars->set_gps_speed_kmph(speed_kmph);

  // Ignoring Mode field

  return 0;
}

/* Parses the specified NMEA sentence and saves the values of interest
 * to the statevars variable
 */
void Gps::parse_gps_sentence(char * sentence) {
  if (strncmp(sentence, GPGGA_START, START_LENGTH) == 0) {
    // ---- DEBUG
    //Serial.println("GPGGA found!\r\n");
    //Serial.println(sentence);
    // Copy the GPGGA sentence to statevars regardless of checksum; and
    // include any null chars as well (versus strcpy)
    // memcpy(statevars.gps_sentence0, sentence, GPS_SENTENCE_BUFF_SZ);

    // Parse the sentence only if the checksum is valid
    if (validate_checksum(sentence) == 1) {
      parse_gpgga(sentence);
      // TODO: Consider changing the macro to STATUS_GPS_VALID_GPGGA_RCVD
      vars->set_status(vars->get_status() | STATUS_GPS_GPGGA_RCVD);
    }
  } else if (strncmp(sentence, GPGSA_START, START_LENGTH) == 0) {
    // ---- DEBUG
    //Serial.println("GPGSA found!\r\n");
    //Serial.println(sentence);

    // memcpy(statevars.gps_sentence1, sentence, GPS_SENTENCE_BUFF_SZ);

    if (validate_checksum(sentence) == 1) {
      // parse_gpgsa(sentence);
      vars->set_status(vars->get_status() | STATUS_GPS_GPGSA_RCVD);
    }
  } else if (strncmp(sentence, GPRMC_START, START_LENGTH) == 0) {
    // ---- DEBUG
    //Serial.println("GPRMC found!\r\n");
    //Serial.println(sentence);

    // memcpy(statevars.gps_sentence2, sentence, GPS_SENTENCE_BUFF_SZ);

    if (validate_checksum(sentence) == 1) {
      parse_gprmc(sentence);
      vars->set_status(vars->get_status() | STATUS_GPS_GPRMC_RCVD);
    }
  } else if (strncmp(sentence, GPVTG_START, START_LENGTH) == 0) {
    // ---- DEBUG
    //Serial.println("GPVTG found!\r\n");
    //Serial.println(sentence);

    // memcpy(statevars.gps_sentence3, sentence, GPS_SENTENCE_BUFF_SZ);

    if (validate_checksum(sentence) == 1) {
      // parse_gpvtg(sentence);
      vars->set_status(vars->get_status() | STATUS_GPS_GPVTG_RCVD);
    }
  } else {
    // We don't care about the GPGSV sentences

    // ---- DEBUG
    //Serial.println("GPGSV ignored\r\n");
  }

  return;
}

/* Validates the checksum of the specified NMEA sentence
 * Returns 1 if calculated checksum matched the received checksum;
 * Returns 0 otherwise
 */
uint8_t Gps::validate_checksum(char * s) {
  uint8_t checksum = 0;
  uint8_t s_cursor;

  // Calculate the checksum of the received characters in the sentence.
  // Note: only the characters between '$' and '*' are used
  for (s_cursor = 1; s_cursor < GPS_SENTENCE_BUFF_SZ; s_cursor++) {
    if (*(s + s_cursor) != '*') {
      checksum ^= *(s + s_cursor);
    } else {
      break;
    }
  }

  // Advance the cursor to the position of the checksum's first char
  s_cursor++;

  // Verify that there was no overflow by checking whether the cursor
  // went beyond the buffer limit, possibly due to not finding a '*'
  if (s_cursor + GPS_CHECKSUM_LENGTH >= GPS_SENTENCE_BUFF_SZ) {
    return 0;
  }

  // Convert the most significant and least significant nibbles of
  // the checksum byte
  uint8_t chk_upper = hexchar_to_dec(s[s_cursor]);
  uint8_t chk_lower = hexchar_to_dec(s[s_cursor + 1]);

  // Verify no error occurred during the conversion
  if (chk_upper == GPS_INVALID_HEX_CHAR || chk_lower == GPS_INVALID_HEX_CHAR) {
    return 0;
  }

  // Assemble the expected checksum provided by the GPS receiver
  uint8_t expected_checksum = (chk_upper << 4) | chk_lower;

  if (checksum == expected_checksum) {
    return 1;
  }

  return 0;
}
