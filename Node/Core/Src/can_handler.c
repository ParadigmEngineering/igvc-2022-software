#include "can_handler.h"
#include "stm32f3xx_hal_can.h"

// TODO probably want some exit codes here for debugging purposes

static CAN_TxHeaderTypeDef txHeader;

void receive_can_message(CanMessage* message)
{
    CAN_RxHeaderTypeDef header;

    if (HAL_CAN_GetRxFifoFillLevel(&hcan, 0) > 0)
    {
        HAL_CAN_GetRxMessage(&hcan, 0, &header, message->data);
        message->id = header.StdId;
        message->len = header.DLC;
    }
}

void send_can_message(uint32_t id, uint8_t* data, uint8_t len)
{
    if (len > 8) { return; }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        txHeader.StdId = id;
        txHeader.ExtId = id;
        txHeader.IDE = CAN_ID_STD;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = len;
        txHeader.TransmitGlobalTime = DISABLE;

        static const uint32_t mailboxes[] = {
            CAN_TX_MAILBOX0,
            CAN_TX_MAILBOX1,
            CAN_TX_MAILBOX2
        };
        // TODO check which mailbox is free and then use it
        // HAL_CAN_AddTxMessage(&hcan, &txHeader, data, uint32_t *pTxMailbox);
    }
}
