#include "state.h"
#include "can_message_defs.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include <stdint.h>
#include "state.h"

// Called in handle_can_messages(), LEDs actuated by state in main
void get_next_state(uint32_t id)
{
  switch(curr_state)
  {
    case STANDBY:
      if (id == AUTONOMOUS_REQUEST_CAN_ID)
      {
        next_state = AUTONOMOUS;
        break;
      }
      if (id == MANUAL_REQUEST_CAN_ID)
      {
        next_state = MANUAL;
        break;
      }
      break;
    case AUTONOMOUS:
      if (id == STANDBY_REQUEST_CAN_ID)
      {
        next_state = STANDBY;
      }
      break;
    case MANUAL:
      if (id == STANDBY_REQUEST_CAN_ID)
      {
        next_state = STANDBY;
      }
      break;
    default:
      next_state = STANDBY;
      break;
  }
}
