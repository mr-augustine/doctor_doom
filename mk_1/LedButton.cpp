/*
 * file: LedButton.cpp
 * created: 20161009
 * author(s): mr-augustine
 */
#include "LedButton.h"

void LedButton::initialize(void) {
  // Turn on the pullup resistors
  BUTTON_PORT |= (1 << BUTTON_PIN);

  // TODO: Can/should the pullup resistor be turned on for an output pin?
  // BUTTON_LED_PORT |= (1 << BUTTON_LED_PIN);

  // Set the button's pin as an input
  BUTTON_DDR &= ~(1 << BUTTON_PIN);

  // Set the button's LED pin as an output
  BUTTON_LED_DDR |= (1 << BUTTON_LED_PIN);

  // Initialize the button state as being not pressed regardless of its
  // current physical state.
  button_state = ButtonState_Unpressed;
  pin_value = NEW_PIN_VALUE;

  enabled = true;
}

int8_t LedButton::verify_init(void) {
  // Verify the pullup resistors were turned on
  // if ( !(BUTTON_PORT & (1 << BUTTON_PIN)) ||
  //      !(BUTTON_LED_PORT & (1 << BUTTON_LED_PIN)) ) {
  if ( !(BUTTON_PORT & (1 << BUTTON_PIN)) ) {
    return 0;
  }

  // Verify button pin is set as an input
  if ( (BUTTON_DDR & (1 << BUTTON_PIN)) ) {
    return -1;
  }

  // Verify LED pin is set as an output
  if ( !(BUTTON_LED_DDR & (1 << BUTTON_LED_PIN)) ) {
    return -2;
  }

  // Verify button state is unpressed
  if ( button_state != ButtonState_Unpressed ) {
    return -3;
  }

  return 1;
}

void LedButton::update(void) {
  if (!enabled) {
    return;
  }

  // Toggle states if the button's pin changed
  if (NEW_PIN_VALUE != pin_value) {

    if (button_state == ButtonState_Unpressed) {
      button_state = ButtonState_Pressed;
    } else {
      button_state = ButtonState_Unpressed;
    }

    pin_value = !pin_value;
  }

  return;
}

bool LedButton::is_pressed(void) {
  if (!enabled) {
    return 0;
  }

  if (button_state == ButtonState_Pressed) {
    return 1;
  }

  return 0;
}

void LedButton::turn_off_led(void) {
  if (!enabled) {
    return;
  }

  BUTTON_LED_PORT &= ~(1 << BUTTON_LED_PIN);

  return;
}

void LedButton::turn_on_led(void) {
  if (!enabled) {
    return;
  }

  BUTTON_LED_PORT |= (1 << BUTTON_LED_PIN);

  return;
}
