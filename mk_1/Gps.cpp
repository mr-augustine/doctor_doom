/*
 * file: Gps.h
 * created: 20161009
 * author(s): mr-augustine
 */
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
