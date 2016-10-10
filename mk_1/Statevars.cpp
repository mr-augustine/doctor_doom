/*
 * file: Statevars.cpp
 * created: 20161009
 * author(s): mr-augustine
 */
#include "Statevars.h"

#include <string.h>

void Statevars::reset(void) {
  memset(this, 0, sizeof(Statevars));

  set_prefix(0xDADAFEED);
  set_suffix(0xCAFEBABE);
}
