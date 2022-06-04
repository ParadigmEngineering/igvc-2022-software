#include "can_handler.h"
#include "bldc_interface.h"
#include "can_message_defs.h"
#include "stm32f334x8.h"
#include "utilities.h"

#include "main.h"

#include <string.h>

#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_gpio.h"

static const float MAX_RPM = 10000.0;
static const float MAX_CURRENT = 20.0;

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

static void motor_control_rpm(uint32_t id, uint8_t data[])
{
    // TODO should probably sanity check the rpm before sending command
    float rpm;
    BldcInterface* motor;
    memcpy(&rpm, data, sizeof(float));

    if (rpm > MAX_RPM || rpm < -MAX_RPM)
    {
        return;
    }

    switch(id)
    {
        case MOTOR_1_RPM:
            motor = &motor1;
            break;
        case MOTOR_2_RPM:
            motor = &motor2;
            break;
        case MOTOR_3_RPM:
            motor = &motor3;
            break;
        default:
            motor = &motor1;
            break;
    }

    bldc_interface_set_rpm(motor, rpm);
}

static void motor_control_current(uint32_t id, uint8_t data[])
{
    // TODO should probably sanity check the rpm before sending command
    float current;
    BldcInterface* motor;
    memcpy(&current, data, sizeof(float));

    if (current > MAX_CURRENT)
    {
        return;
    }

    switch(id)
    {
        case MOTOR_1_CURRENT:
            motor = &motor1;
            break;
        case MOTOR_2_CURRENT:
            motor = &motor2;
            break;
        case MOTOR_3_CURRENT:
            motor = &motor3;
            break;
        default:
            motor = &motor1;
            break;
    }

    bldc_interface_set_current(motor, current);
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
            /* Okay so here was what I was thinking for the motor control logic, something along these lines at least.
            * - We know that Colton wants to be able to control each motor individually, as well as have different ids
            *   based on the state
            *  Here goes some rough logic:
            *  if (MOTOR1_CONTROL_RPM_MASK)
            *  {
            *      if (MOTOR1_CONTROL_RPM_AUTO && curr_state == AUTONOMOUS)
            *      {
            *          motor1_control_rpm_auto(message.data);
            *      }
            *      if (MOTOR1_CONTROL_RPM_MANUAL && curr_state == MANUAL);
            *      {
            *          motor1_control_rpm_manual(message.data);
            *      }
            *  }
            *  ... Continue for three motors, making 6 funcs total!
            */
            if (message.id & STATE_CHANGE_CAN_ID)
            {
                get_next_state(message.id);
            }
            
            if (message.id & MOTOR_CONTROL_RPM_MASK)
            {
                motor_control_rpm(message.id, message.data);
            }
            else if (message.id & MOTOR_CONTROL_CURRENT_MASK)
            {
                motor_control_current(message.id, message.data);
            }

            // Gonna remove this soon, this is covered in state transitions and main
            else if (message.id & LED_ON_TEST_MASK)
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
