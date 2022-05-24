#include <can.h>

typedef struct CanMessage
{
    uint32_t id;
    uint32_t len;
    uint8_t data[8];
} CanMessage;

typedef enum CanStatus
{
    CAN_GOOD,
    CAN_RX_FIFO_FULL,
    CAN_TX_MAILBOXES_FULL,
    CAN_DATA_TOO_LONG,
} CanStatus;

CanStatus receive_can_message(CanMessage* message);
CanStatus send_can_message(CanMessage* message);
