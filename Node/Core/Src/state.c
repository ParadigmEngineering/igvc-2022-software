#include "state.h"
#include "can_message_defs.h"

// Called in handle_can_messages(), LEDs actuated by state in main
void get_next_state(uint32_t id)
{
  switch(curr_state)
  {
    case BOOT:
      if (1)
      {
        next_state = STANDBY;
        break;
      }
    case STANDBY:
      if (id == AUTONOMOUS_CAN_ID)
      {
        next_state = AUTONOMOUS;
        break;
      }
      if (id == MANUAL_CAN_ID)
      {
        next_state = MANUAL;
        break;
      }
      break;
    case AUTONOMOUS:
      if (id == STANDBY_CAN_ID)
      {
        next_state = STANDBY;
        break;
      }
      else if (id != AUTONOMOUS_CAN_ID)
      {
        next_state = STANDBY;
        break;
      }
      break;
    case MANUAL:
      if (id == STANDBY_CAN_ID)
      {
        next_state = STANDBY;
        break;
      }
      else if (id != MANUAL_CAN_ID)
      {
        next_state = STANDBY;
        break;
      }
      break;
    default:
      next_state = STANDBY;
      break;
  }
}
