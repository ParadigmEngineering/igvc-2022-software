#include "can_handler.h"
#include "utilities.h"
#include "stm32f3xx_hal_can.h"

static CAN_TxHeaderTypeDef txHeader;

CanStatus receive_can_message(CanMessage* message)
{
    CAN_RxHeaderTypeDef header;

    if (HAL_CAN_GetRxFifoFillLevel(&hcan, 0) == 0)
    {
        return CAN_RX_FIFO_FULL;
    }

    HAL_CAN_GetRxMessage(&hcan, 0, &header, message->data);
    message->id = header.StdId;
    message->len = header.DLC;

    return CAN_GOOD;
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
