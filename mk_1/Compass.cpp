/*
 * file: Compass.cpp
 * created: 20161010
 * author(s): mr-augustine
 */
#include "Compass.h"

/* We're using an unterminated do-while loop so that we can use a semicolon
 * when invoking the macro in our update_xxx() functions. It's for the sake of
 * readability below.
 */
#define INIT_READING   \
do { TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWSTA); } \
while (0)

static volatile uint16_t heading_reading = 0;
static volatile uint8_t heading_ready = 0;

static volatile uint8_t pitch_reading = 0;
static volatile uint8_t pitch_ready = 0;

static volatile uint8_t roll_reading = 0;
static volatile uint8_t roll_ready = 0;

static volatile uint8_t compass_active = 0;
static volatile uint8_t compass_error = 0;

static volatile uint8_t requested_register = CmpsReg_Heading_High;
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
        case CmpsReg_Heading_High:
        //case CmpsReg_Heading_Low:
        //case CmpsReg_Pitch:
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
          break;
        case CmpsReg_Roll:
          //TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
          //break;
        case CmpsReg_Heading_Low:
        case CmpsReg_Pitch:
        default:
          //uwrite_print_buff("*******TW_MR_SLA_ACK Error *******\r\n");
          return;
      }
      break;

    // Compass sent data and AVR sent an ACK; read the next incoming byte
    // from the data register and expect another byte to follow
    case TW_MR_DATA_ACK:
      switch (requested_register) {
        case CmpsReg_Heading_High:
          heading_reading = TWDR;
          heading_reading = heading_reading << 8;
          requested_register = CmpsReg_Heading_Low;
          break;
        case CmpsReg_Heading_Low:
          heading_reading |= TWDR;
          heading_ready = 1;
          requested_register = CmpsReg_Pitch;
          break;
        case CmpsReg_Pitch:
          pitch_reading = TWDR;
          pitch_ready = 1;
          requested_register = CmpsReg_Roll;
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
          return;
        // This case should not occur because we expect the AVR to have sent
        // a NACK after the roll value was received
        case CmpsReg_Roll:
          //uwrite_print_buff("***** Register Roll! *****\r\n");
          break;
        default:
          //uwrite_print_buff("*********TW_MR_DATA_ACK ERROR********\r\n");
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
        case CmpsReg_Roll:
          roll_reading = TWDR;
          roll_ready = 1;
          compass_active = 0;
          break;
        // These cases should not occur because we expect the AVR to have sent
        // an ACK after these values were received
        case CmpsReg_Heading_High:
        case CmpsReg_Heading_Low:
        case CmpsReg_Pitch:
        default:
          //uwrite_print_buff("*********TW_MR_DATA_NACK ERROR********\r\n");
          compass_error = 1;
          heading_reading = 0xEEEE;   // 0xE is for error
          compass_active = 0;
          TWCR = (1 << TWSTO) | (1 << TWEN);
          return;
      }

      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
      break;
    default:
      //uwrite_print_buff("*********SWITCH ERROR********\r\n");
      compass_error = 1;
      heading_reading = 0xEEEE;   // 0xE is for error
      pitch_reading = 0xBB;       // 0xB is for bad
      roll_reading = 0xBB;
      compass_active = 0;
      TWCR = (1 << TWSTO) | (1 << TWEN);
  } // switch (status)
}

Compass::Compass(const Statevars * s) {
  vars = s;
}

void Compass::initialize(void) {
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
}

int8_t Compass::verify_init(void) {
  // Verify that the SCL clock fequency is set to 100 kHz
  if (TWBR != 0x48) {
    return 0;
  }

  // Verify that the two wire interface is enabled
  if ( !(TWCR & (1 << TWEN)) ) {
    return -1;
  }

  // Verify that TWI interrupts are enabled
  if ( !(TWCR & (1 << TWIE)) ) {
    return -2;
  }

  return 1;
}

void Compass::update(void) {
  if (compass_active) {
    return;
  }

  if (heading_ready) {
    //cmps10_heading = heading_reading;
    vars->set_heading_raw(heading_reading);
    vars->set_heading_deg(heading_reading / 10.0);
  }

  if (pitch_ready) {
    //cmps10_pitch = pitch_reading;
    vars->set_pitch_deg(pitch_reading);
  }

  if (roll_ready) {
    //cmps10_roll = roll_reading;
    vars->set_roll_deg(roll_reading);
  }

  if (compass_error) {
    // TODO Save the error to statevars
  }

  requested_register = CmpsReg_Heading_High;
  begin_new_reading();

  return;
}

void Compass::begin_new_reading(void) {
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
