#ifndef STATE_H
#define STATE_H

#include <stdint.h>

typedef enum
{
  STANDBY,
  AUTONOMOUS,
  MANUAL,
  BOOT
} state;

void get_next_state(uint32_t id);

extern state curr_state;
extern state next_state;

#endif