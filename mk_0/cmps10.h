#ifndef _CMPS10_H_
#define _CMPS10_H_

#include <avr/interrupt.h>
#include "statevars.h"
#include "twi.h"

/* We're using an unterminated do-while loop so that we can use a semicolon
 * when invoking the macro in our update_xxx() functions. It's for the sake of
 * readability below.
 */
#define INIT_READING   \
do { TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA); } \
while (0)

#define COMPASS_ADDR          0x60
#define COMPASS_HEADING_REG   2
#define COMPASS_PITCH_REG     4
#define COMPASS_ROLL_REG      5

enum Cmps_Reg {
  Register_Heading_High = 0,
  Register_Heading_Low,
  Register_Pitch,
  Register_Roll
};

static volatile uint16_t heading_reading = 0;
static volatile uint8_t heading_ready = 0;

static volatile uint8_t pitch_reading = 0;
static volatile uint8_t pitch_ready = 0;

static volatile uint8_t roll_reading = 0;
static volatile uint8_t roll_ready = 0;

static volatile uint8_t compass_active = 0;
static volatile uint8_t compass_error = 0;

static volatile uint8_t requested_register = Register_Heading_High;
static uint8_t compass_enabled;

ISR(TWI_vect) {
  uint8_t status = TW_STATUS;

  switch (status) {
    // AVR sent start sequence; now send the compass address + write
    case TW_START:
      TWDR = (COMPASS_ADDR << 1) | TW_WRITE;
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
      break;

    /* Compass received the address; now send the register address for
     * the value we're interested in. Here we use the starting address
     * COMPASS_HEADING_REG since it is the smallest of the four addresses
     * and the compass will automatically increment to the next higher
     * addresses [i.e. heading MSB (2), heading LSB (3) , pitch (4), roll(5)]
     */
    case TW_MT_SLA_ACK:
      TWDR = COMPASS_HEADING_REG;
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
      break;

    /* Compass received the address of the register we want;
     * now send a repeated start
     */
    case TW_MT_DATA_ACK:
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA);
      break;

    // AVR sent the repeated start; now send the compass address + read
    case TW_REP_START:
      TWDR = (COMPASS_ADDR << 1) | TW_READ;
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
      break;

    // Compass acknowledged the read request, it's ready to send;
    // AVR will send clock cycles to read the data from the
    // compass into the data register then (1) send an acknowledgment
    // to continue reading the available values, or (2) send no acknowledgement
    // when we received the last value we wanted.
    case TW_MR_SLA_ACK:
      switch (requested_register) {
        case Register_Heading_High:
        //case Register_Heading_Low:
        //case Register_Pitch:
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
          break;
        case Register_Roll:
          //TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
          //break;
        case Register_Heading_Low:
        case Register_Pitch:
        default:
          Serial.println("*******TW_MR_SLA_ACK Error *******");
          return;
      }
      break;

    // Compass sent data and AVR sent an ACK; read the next incoming byte
    // from the data register and expect another byte to follow
    case TW_MR_DATA_ACK:
      switch (requested_register) {
        case Register_Heading_High:
          heading_reading = TWDR;
          heading_reading = heading_reading << 8;
          requested_register = Register_Heading_Low;
          break;
        case Register_Heading_Low:
          heading_reading |= TWDR;
          heading_ready = 1;
          requested_register = Register_Pitch;
          break;
        case Register_Pitch:
          pitch_reading = TWDR;
          pitch_ready = 1;
          requested_register = Register_Roll;
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
          return;
        // This case should not occur because we expect the AVR to have sent
        // a NACK after the roll value was received
        case Register_Roll:
          Serial.println("***** Register Roll! *****");
          break;
        default:
          Serial.println("*********TW_MR_DATA_ACK ERROR********");
          compass_error = 1;
          heading_reading = 0xEEEE;   // 0xE is for error
          pitch_reading = 0xBB;       // 0xB is for bad
          roll_reading = 0xBB;
          compass_active = 0;
          TWCR = (1 << TWSTO) | (1 << TWEN);
          return;
      }

      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
      break;

    /* Master received data and didn't send an ACK
     * Roll received; now read the data and send a NACK
     */
    case TW_MR_DATA_NACK:
      switch (requested_register) {
        case Register_Roll:
          roll_reading = TWDR;
          roll_ready = 1;
          compass_active = 0;
          break;
        // These cases should not occur because we expect the AVR to have sent
        // an ACK after these values were received
        case Register_Heading_High:
        case Register_Heading_Low:
        case Register_Pitch:
        default:
          Serial.println("*********TW_MR_DATA_NACK ERROR********");
          compass_error = 1;
          heading_reading = 0xEEEE;   // 0xE is for error
          compass_active = 0;
          TWCR = (1 << TWSTO) | (1 << TWEN);
          return;
      }

      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
      break;
    default:
      Serial.println("*********SWITCH ERROR********");
      compass_error = 1;
      heading_reading = 0xEEEE;   // 0xE is for error
      pitch_reading = 0xBB;       // 0xB is for bad
      roll_reading = 0xBB;
      compass_active = 0;
      TWCR = (1 << TWSTO) | (1 << TWEN);
  } // switch (status)
}


uint8_t cmps10_init(void);
void cmps10_update_all(void);

/* Helper function to initiate a new compass reading. Resets the
 * helper variables and triggers the compass.
 */
static void begin_new_reading(void) {
  // Reset all variables in preparation for a new reading
  heading_reading = 0xFFFF;
  heading_ready = 0;

  pitch_reading = 0xFF;
  pitch_ready = 0;

  roll_reading = 0xFF;
  roll_ready = 0;

  compass_error = 0;
  compass_active = 1;

  if (!compass_enabled) {
    return;
  }

  // Initiate a new compass reading
  INIT_READING;

  return;
}

/* Initialzes the compass by enabling the Two-Wire Interface (TWI), and
 * sets the SCL clock frequency to 100 kHz.
 */
uint8_t cmps10_init(void) {
  compass_active = 0;
  compass_error = 0;

  heading_reading = 0;
  heading_ready = 0;

  pitch_reading = 0;
  pitch_ready = 0;

  roll_reading = 0;
  roll_ready = 0;


  // Set the SCL clock frequency to 100 kHz
  // See Section 22.5.2 in the Atmel Specsheet for the formula
  TWBR = 0x48;

  // Enable the two wire interface and enable interrupts
  TWCR = (1 << TWEN) | (1 << TWIE);

  compass_enabled = 1;

  return compass_enabled;
}

/* Requests the heading, pitch, and roll measurements from the compass
 */
void cmps10_update_all(void) {
  if (compass_active) {
    return;
  }

  if (heading_ready) {
    //cmps10_heading = heading_reading;
    statevars.heading_raw = heading_reading;
    statevars.heading_deg = heading_reading / 10.0;
  }

  if (pitch_ready) {
    //cmps10_pitch = pitch_reading;
    statevars.pitch_deg = pitch_reading;
  }

  if (roll_ready) {
    //cmps10_roll = roll_reading;
    statevars.roll_deg = roll_reading;
  }

  if (compass_error) {
    // TODO Save the error to statevars
  }

  requested_register = Register_Heading_High;
  begin_new_reading();

  return;
}

#endif
