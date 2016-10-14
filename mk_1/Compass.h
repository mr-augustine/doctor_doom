/*
 * file: Compass.h
 * created: 20161010
 * author(s): mr-augustine
 *
 * Describes the class used to initialize and update the compass (CMPS10).
 */
#ifndef _COMPASS_H_
#define _COMPASS_H_

#include <avr/interrupt.h>
#include <HardwareSerial.h>
#include "Statevars.h"
#include "twi.h"

#define COMPASS_ADDR          0x60
#define COMPASS_HEADING_REG   2
#define COMPASS_PITCH_REG     4
#define COMPASS_ROLL_REG      5

enum CompassRegister {
  CmpsReg_Heading_High = 0,
  CmpsReg_Heading_Low,
  CmpsReg_Pitch,
  CmpsReg_Roll
};

class Compass {
private:
  Statevars * vars;
  uint8_t enabled;

  void begin_new_reading(void);

public:
  Compass(const Statevars * s);
  void initialize(void);
  int8_t verify_init(void);
  void update(void);
};

#endif
