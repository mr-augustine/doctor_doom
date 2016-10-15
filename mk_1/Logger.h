/*
 * file: Logger.h
 * created: 20161010
 * author(s): mr-augustine
 *
 * Describes the class used to log the robot's state variables.
 */
#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <HardwareSerial.h>
#include <SD.h>

#include "Pins.h"
#include "Robot.h"
#include "Statevars.h"

class Logger {
private:
  Statevars * data;
  uint32_t data_size;
  File data_file;

  uint8_t init_datafile(void);

public:
  Logger(const Statevars * s);
  void initialize(void);
  int8_t verify_init(void);
  void write(void);
  void finish(void);
};

#endif // #ifndef _LOGGER_H_
