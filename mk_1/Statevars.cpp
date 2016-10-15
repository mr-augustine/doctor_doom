/*
 * file: Statevars.cpp
 * created: 20161009
 * author(s): mr-augustine
 */
#include "Statevars.h"

#include <string.h>

void Statevars::initialize(void) {
  memset(this, 0, sizeof(Statevars));

  set_prefix(DEFAULT_PREFIX);
  set_suffix(DEFAULT_SUFFIX);
}

int8_t Statevars::verify_init(void) {
  return (prefix == DEFAULT_PREFIX && suffix == DEFAULT_SUFFIX);
}
