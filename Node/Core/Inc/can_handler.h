#include <can.h>

typedef struct CanMessage
{
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} CanMessage;

typedef enum CanStatus
{
    CAN_GOOD,
    CAN_RX_FIFO_EMPTY,
    CAN_TX_MAILBOXES_FULL,
    CAN_DATA_TOO_LONG,
} CanStatus;

CanStatus receive_can_message(CanMessage* message);
CanStatus send_can_message(CanMessage* message);
void handle_can_messages(uint8_t num_msgs_to_handle);
