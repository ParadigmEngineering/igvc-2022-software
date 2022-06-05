#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void motor_control_rpm(uint32_t id, uint8_t data[]);
void motor_control_current(uint32_t id, uint8_t data[]);
void motor_control_duty_cycle(uint32_t id, uint8_t data[]);

#endif