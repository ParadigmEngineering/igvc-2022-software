#include "can_handler.h"
#include "can_message_defs.h"
#include "stm32f334x8.h"
#include "utilities.h"

#include "main.h"

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

void handle_can_messages(uint8_t num_msgs_to_handle)
{
    CanMessage message;
    CanStatus status;

    for (int i = 0; i < num_msgs_to_handle; i++)
    {
        status = receive_can_message(&message);
        if (status == CAN_GOOD)
        {
            if (message.id & LED_ON_TEST_MASK)
            {
                led_test(message.id, GPIO_PIN_SET);
            }
            else if (message.id & LED_OFF_TEST_MASK)
            {
                led_test(message.id, GPIO_PIN_RESET);
            }
        }
        else if (status == CAN_RX_FIFO_EMPTY) { break; }
    }
}
