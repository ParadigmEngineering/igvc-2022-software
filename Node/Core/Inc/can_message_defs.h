#define HEARTBEAT_ID        0x0001

#define STATE_CHANGE_CAN_ID          0x0010
#define STANDBY_REQUEST_CAN_ID       0x0011
#define AUTONOMOUS_REQUEST_CAN_ID    0x0012
#define MANUAL_REQUEST_CAN_ID        0x0013
#define BOOT_RESPONSE_CAN_ID         0x0016
#define STANDBY_RESPONSE_CAN_ID      0x0017
#define AUTONOMOUS_RESPONSE_CAN_ID   0x0018
#define MANUAL_RESPONSE_CAN_ID       0x0019

#define NODE_GOOD_CAN_ID_MASK 0x0014
#define NODE_BAD_CAN_ID_MASK  0x0015

#define MOTOR_CONTROL_RPM_MASK_MANUAL 0x0020
#define MOTOR_1_RPM_MANUAL            0x0021
#define MOTOR_2_RPM_MANUAL            0x0022
#define MOTOR_3_RPM_MANUAL            0x0023

#define MOTOR_CONTROL_RPM_MASK_AUTONOMOUS 0x0040
#define MOTOR_1_RPM_AUTONOMOUS            0x0041
#define MOTOR_2_RPM_AUTONOMOUS            0x0042
#define MOTOR_3_RPM_AUTONOMOUS            0x0043

#define IS_MOTOR1_RPM_CONTROL(motor) ((motor == MOTOR_1_RPM_AUTONOMOUS) || (motor == MOTOR_1_RPM_MANUAL))
#define IS_MOTOR2_RPM_CONTROL(motor) ((motor == MOTOR_2_RPM_AUTONOMOUS) || (motor == MOTOR_2_RPM_MANUAL))
#define IS_MOTOR3_RPM_CONTROL(motor) ((motor == MOTOR_3_RPM_AUTONOMOUS) || (motor == MOTOR_3_RPM_MANUAL))

#define MOTOR_CONTROL_CURRENT_MASK_MANUAL 0x0080
#define MOTOR_1_CURRENT_MANUAL            0x0081
#define MOTOR_2_CURRENT_MANUAL            0x0082
#define MOTOR_3_CURRENT_MANUAL            0x0083

#define MOTOR_CONTROL_CURRENT_MASK_AUTONOMOUS 0x0100
#define MOTOR_1_CURRENT_AUTONOMOUS            0x0101
#define MOTOR_2_CURRENT_AUTONOMOUS            0x0102
#define MOTOR_3_CURRENT_AUTONOMOUS            0x0103

#define IS_MOTOR1_CURRENT_CONTROL(motor) ((motor == MOTOR_1_CURRENT_AUTONOMOUS) || (motor == MOTOR_1_CURRENT_MANUAL))
#define IS_MOTOR2_CURRENT_CONTROL(motor) ((motor == MOTOR_2_CURRENT_AUTONOMOUS) || (motor == MOTOR_2_CURRENT_MANUAL))
#define IS_MOTOR3_CURRENT_CONTROL(motor) ((motor == MOTOR_3_CURRENT_AUTONOMOUS) || (motor == MOTOR_3_CURRENT_MANUAL))

#define MOTOR_CONTROL_DUTY_CYCLE_MASK_MANUAL 0x0200
#define MOTOR_1_DUTY_CYCLE_MANUAL            0x0201
#define MOTOR_2_DUTY_CYCLE_MANUAL            0x0202
#define MOTOR_3_DUTY_CYCLE_MANUAL            0x0203

#define MOTOR_CONTROL_DUTY_CYCLE_MASK_AUTONOMOUS 0x0400
#define MOTOR_1_DUTY_CYCLE_AUTONOMOUS            0x0401
#define MOTOR_2_DUTY_CYCLE_AUTONOMOUS            0x0402
#define MOTOR_3_DUTY_CYCLE_AUTONOMOUS            0x0403

#define IS_MOTOR1_DUTY_CYCLE_CONTROL(motor) ((motor == MOTOR_1_DUTY_CYCLE_AUTONOMOUS) || (motor == MOTOR_1_DUTY_CYCLE_MANUAL))
#define IS_MOTOR2_DUTY_CYCLE_CONTROL(motor) ((motor == MOTOR_2_DUTY_CYCLE_AUTONOMOUS) || (motor == MOTOR_2_DUTY_CYCLE_MANUAL))
#define IS_MOTOR3_DUTY_CYCLE_CONTROL(motor) ((motor == MOTOR_3_DUTY_CYCLE_AUTONOMOUS) || (motor == MOTOR_3_DUTY_CYCLE_MANUAL))

#define WIRELESS_ESTOP  120

#define LED_1_TEST_ON    0x0002
#define LED_2_TEST_ON    0x0003
#define LAMP_1_TEST_ON   0x0004
#define LAMP_2_TEST_ON   0x0005
#define LAMP_3_TEST_ON   0x0006
#define LAMP_4_TEST_ON   0x0007
#define LED_1_TEST_OFF   0x0008
#define LED_2_TEST_OFF   0x0009
#define LAMP_1_TEST_OFF  0x000a
#define LAMP_2_TEST_OFF  0x000b
#define LAMP_3_TEST_OFF  0x000c
#define LAMP_4_TEST_OFF  0x000d
