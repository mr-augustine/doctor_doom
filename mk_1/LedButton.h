/*
 * file: LedButton.h
 * created: 20161009
 * author(s): mr-augustine
 *
 * Declares the class used for a SPST pushbutton that has an LED indicator.
 * The actual button I used for this was from Radio Shack (# 275-0009).
 *
 * Regardless of the button's initial physical state (pressed or unpressed),
 * this library will initialize the button as being unpressed.
 */
#ifndef _LED_BUTTON_H_
#define _LED_BUTTON_H_

#include <avr/io.h>
#include "LedButton.h"
#include "Pins.h"

#define NEW_PIN_VALUE   ((BUTTON_PINVEC & (1 << BUTTON_PIN)) >> BUTTON_PIN)

enum ButtonState {
  ButtonState_Unpressed = 0,
  ButtonState_Pressed,
};

class LedButton {
private:
  bool enabled;
  volatile ButtonState button_state;
  uint8_t pin_value;

  void initialize(void);

public:
  LedButton(void);
  int8_t verify_init(void);
  void update(void);
  bool is_pressed(void);
  void turn_off_led(void);
  void turn_on_led();
};

 #endif // #ifndef _LED_BUTTON_H_
