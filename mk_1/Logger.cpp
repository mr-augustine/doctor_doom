/*
 * file: Logger.cpp
 * created: 20161010
 * author(s): mr-augustine
 */
#include "Logger.h"

Logger::Logger(const Statevars * s) {
  data = s;
  data_size = sizeof(Statevars);
}

void Logger::initialize(void) {
  pinMode(SDCARD_CHIP_SELECT, OUTPUT);

  if (!SD.begin(SDCARD_CHIP_SELECT)) {
    Serial.println("SD Card didn't initialize");
    return;
  }

  if (!init_datafile()) {
    Serial.println("Could not start a new datafile");
    return;
  }

  return;
}

uint8_t Logger::init_datafile(void) {
  char filepath[32];

  // Ensure that a folder with the same name as the robot is created
  if (!SD.exists(ROBOT_NAME)) {
    SD.mkdir(ROBOT_NAME);
  }

  // Create the path to the new file
  // File names must be in the 8.3 format (i.e., 8 characters for the file name
  // and 3 characters for the file extension)
  uint16_t file_index;
  for (file_index = 1; file_index < UINT16_MAX; file_index++) {
    snprintf(filepath, sizeof(filepath), "/%s/k%05u.dat",
              ROBOT_NAME, file_index);

    if (!SD.exists(filepath)) {
      data_file = SD.open(filepath, FILE_WRITE);
      break;
    }
  }
  if (file_index == UINT16_MAX) {
    return 0;
  }

  if (!data_file) {
    return 0;
  }

  Serial.print(filepath);
  Serial.println(" was opened for write!");

  return 1;
}
