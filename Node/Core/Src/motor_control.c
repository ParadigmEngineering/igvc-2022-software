#include "bldc_interface.h"
#include "can_message_defs.h"
#include "main.h"
#include "motor_control.h"

#include <string.h>

static const float MAX_RPM = 10000.0;
static const float MAX_CURRENT = 20.0;
static const float MAX_DUTY_CYCLE = 0.05;

void motor_control_rpm(uint32_t id, uint8_t data[])
{
    float rpm;
    BldcInterface* motor;
    memcpy(&rpm, data, sizeof(float));

    if (rpm > MAX_RPM || rpm < -MAX_RPM)
    {
        return;
    }

    if (IS_MOTOR1_RPM_CONTROL(id))
    {
        motor = &motor1;
    }
    else if (IS_MOTOR2_RPM_CONTROL(id))
    {
        motor = &motor2;
    }
    else if (IS_MOTOR3_RPM_CONTROL(id))
    {
        motor = &motor3;
    }
    else
    {
        return;
    }

    bldc_interface_set_rpm(motor, rpm);
}

void motor_control_current(uint32_t id, uint8_t data[])
{
    float current;
    BldcInterface* motor;
    memcpy(&current, data, sizeof(float));

    if (current > MAX_CURRENT || current < -MAX_CURRENT)
    {
        return;
    }

    if (IS_MOTOR1_CURRENT_CONTROL(id))
    {
        motor = &motor1;
    }
    else if (IS_MOTOR2_CURRENT_CONTROL(id))
    {
        motor = &motor2;
    }
    else if (IS_MOTOR3_CURRENT_CONTROL(id))
    {
        motor = &motor3;
    }
    else
    {
        return;
    }

    bldc_interface_set_current(motor, current);
}

void motor_control_duty_cycle(uint32_t id, uint8_t data[])
{
    float duty_cycle;
    BldcInterface* motor;
    memcpy(&duty_cycle, data, sizeof(float));

    if (duty_cycle > MAX_DUTY_CYCLE || duty_cycle < -MAX_DUTY_CYCLE)
    {
        return;
    }

    if (IS_MOTOR1_DUTY_CYCLE_CONTROL(id))
    {
        motor = &motor1;
    }
    else if (IS_MOTOR2_DUTY_CYCLE_CONTROL(id))
    {
        motor = &motor2;
    }
    else if (IS_MOTOR3_DUTY_CYCLE_CONTROL(id))
    {
        motor = &motor3;
    }
    else
    {
        return;
    }

    bldc_interface_set_duty_cycle(motor, duty_cycle);
}