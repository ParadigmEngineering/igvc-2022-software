#include <can.h>

typedef struct CanMessage
{
    uint32_t id;
    uint32_t len;
    uint8_t data[256];
} CanMessage;

void receive_can_message();
void send_can_message(uint32_t id, uint8_t* data, uint8_t len);

