#include "can_handler.h"
#include "can_message_defs.h"
#include "main.h"
#include "stm32f3xx_hal_gpio.h"
#include "utilities.h"
#include "stm32f3xx_hal_can.h"

static CAN_TxHeaderTypeDef txHeader;

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

    HAL_CAN_AddTxMessage(&hcan, &txHeader, message->data, NULL);
    return CAN_GOOD;
}

void handle_can_messages(uint8_t num_msgs_to_handle)
{
    CanMessage message;
    CanStatus status;

    if ((status = receive_can_message(&message)) == CAN_GOOD)
    {
        switch (message.id)
        {
        // TODO maybe some fancy bit manipulation of IDs to make
        // this code a bit nicer
        case LED_1_TEST_ON:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
            break;
        case LED_2_TEST_ON:
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
            break;
        case LAMP_1_TEST_ON:
            HAL_GPIO_WritePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin, GPIO_PIN_SET);
            break;
        case LAMP_2_TEST_ON:
            HAL_GPIO_WritePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin, GPIO_PIN_SET);
            break;
        case LAMP_3_TEST_ON:
            HAL_GPIO_WritePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin, GPIO_PIN_SET);
            break;
        case LAMP_4_TEST_ON:
            HAL_GPIO_WritePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin, GPIO_PIN_SET);
            break;

        case LED_1_TEST_OFF:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            break;
        case LED_2_TEST_OFF:
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            break;
        case LAMP_1_TEST_OFF:
            HAL_GPIO_WritePin(LAMP1_ON_GPIO_Port, LAMP1_ON_Pin, GPIO_PIN_RESET);
            break;
        case LAMP_2_TEST_OFF:
            HAL_GPIO_WritePin(LAMP2_ON_GPIO_Port, LAMP2_ON_Pin, GPIO_PIN_RESET);
            break;
        case LAMP_3_TEST_OFF:
            HAL_GPIO_WritePin(LAMP3_ON_GPIO_Port, LAMP3_ON_Pin, GPIO_PIN_RESET);
            break;
        case LAMP_4_TEST_OFF:
            HAL_GPIO_WritePin(LAMP4_ON_GPIO_Port, LAMP4_ON_Pin, GPIO_PIN_RESET);
            break;
        }
    }

}




