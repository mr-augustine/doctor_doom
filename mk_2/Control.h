#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <math.h>

#include "Statevars.h"
#include "Mobility.h"

#define K_PROP                      2.5 // proportional gain
#define K_RATE                      0 // derivative gain
#define K_INTEGRAL                  0 // integral gain

#define TARGET_HEADING 270.0

class Control {
private:
  Statevars * vars;

  float xtrack_error;
  float xtrack_error_prev;
  float xtrack_error_rate;
  float xtrack_error_sum;
  float steer_control = STEER_NEUTRAL;

  void update_xtrack_error(void);
  void update_xtrack_error_rate(void);
  void update_xtrack_error_sum(void);

public:
  Control(const Statevars * s);
  void update(void);
};

#endif
