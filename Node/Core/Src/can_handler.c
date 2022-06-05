#include "can_handler.h"
#include "bldc_interface.h"
#include "can_message_defs.h"
#include "stm32f334x8.h"
#include "state.h"
#include "stm32f3xx_hal.h"
#include "utilities.h"
#include "motor_control.h"

#include "main.h"

#include <string.h>

#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_gpio.h"

static CAN_TxHeaderTypeDef txHeader;

static void led_test(uint32_t id, GPIO_PinState state)
{
    GPIO_TypeDef* port;
    uint16_t pin;

    switch(id)
    {
    case LED_1_TEST_ON:
    case LED_1_TEST_OFF:
        port = LED1_GPIO_Port;
        pin = LED1_Pin;
        break;
    case LED_2_TEST_ON:
    case LED_2_TEST_OFF:
        port = LED2_GPIO_Port;
        pin = LED2_Pin;
        break;
    case LAMP_1_TEST_ON:
    case LAMP_1_TEST_OFF:
        port = LAMP1_ON_GPIO_Port;
        pin = LAMP1_ON_Pin;
        break;
    case LAMP_2_TEST_ON:
    case LAMP_2_TEST_OFF:
        port = LAMP2_ON_GPIO_Port;
        pin = LAMP2_ON_Pin;
        break;
    case LAMP_3_TEST_ON:
    case LAMP_3_TEST_OFF:
        port = LAMP3_ON_GPIO_Port;
        pin = LAMP3_ON_Pin;
        break;
    case LAMP_4_TEST_ON:
    case LAMP_4_TEST_OFF:
        port = LAMP4_ON_GPIO_Port;
        pin = LAMP4_ON_Pin;
        break;
    default:
        return;
    }

    HAL_GPIO_WritePin(port, pin, state);
}

CanStatus receive_can_message(CanMessage* message)
{
    CAN_RxHeaderTypeDef header;

    uint32_t rx_fifos[] = { CAN_RX_FIFO0, CAN_RX_FIFO1 };

    for (int i = 0; i < ARRAY_SIZE(rx_fifos); i++)
    {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, rx_fifos[i]) != 0)
        {
            HAL_CAN_GetRxMessage(&hcan, rx_fifos[i], &header, message->data);
            message->id = header.StdId;
            message->len = header.DLC;

            return CAN_GOOD;
        }
    }

    return CAN_RX_FIFO_EMPTY;
}

// Won't send message if Tx mailbox is full
CanStatus send_can_message(CanMessage* message)
{
    if (message->len > 8) { return CAN_DATA_TOO_LONG; }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        return CAN_TX_MAILBOXES_FULL;
    }

    txHeader.StdId = message->id;
    txHeader.ExtId = message->id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = message->len;
    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;

    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, message->data, &mailbox) != HAL_OK)
    {
        return CAN_TX_MAILBOXES_FULL;
    }

    return CAN_GOOD;
}

// Will block and try to send CAN message
CanStatus send_can_message_blocking(CanMessage* message)
{
    CanStatus status;
    do { status = send_can_message(message); }
    while (status == CAN_TX_MAILBOXES_FULL);

    return status;
}

void handle_can_messages(uint8_t num_msgs_to_handle)
{
    CanMessage message;
    CanStatus status;

    for (int i = 0; i < num_msgs_to_handle; i++)
    {
        status = receive_can_message(&message);
        if (status == CAN_GOOD)
        {
            if (message.id & HEARTBEAT_ID)
            {
                last_heartbeat_received = HAL_GetTick();
            }
            else if ((message.id & MOTOR_CONTROL_RPM_MASK_MANUAL) &&
                     curr_state == MANUAL)
            {
                motor_control_rpm(message.id, message.data);
            }
            else if ((message.id & MOTOR_CONTROL_RPM_MASK_AUTONOMOUS) &&
                     curr_state == AUTONOMOUS)
            {
                motor_control_current(message.id, message.data);
            }
            else if ((message.id & MOTOR_CONTROL_CURRENT_MASK_MANUAL) &&
                     curr_state == MANUAL)
            {
                motor_control_rpm(message.id, message.data);
            }
            else if ((message.id & MOTOR_CONTROL_CURRENT_MASK_AUTONOMOUS) &&
                     curr_state == AUTONOMOUS)
            {
                motor_control_current(message.id, message.data);
            }
            else if (message.id & STATE_CHANGE_CAN_ID)
            {
                get_next_state(message.id);
            }
            else if (message.id < 0x7)
            {
                led_test(message.id, GPIO_PIN_SET);
            }
            else if (message.id > 0x7 && message.id < 0xe)
            {
                led_test(message.id, GPIO_PIN_RESET);
            }
        }
        else if (status == CAN_RX_FIFO_EMPTY) { break; }
    }
}
