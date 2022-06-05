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
    case BOOT:
      if (vesc_data_valid[0] && vesc_data_valid[1] && vesc_data_valid[2])
      {
        next_state = STANDBY;
        last_heartbeat_received = HAL_GetTick();
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
