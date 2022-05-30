/*
	Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "datatypes.h"

typedef struct BldcInterface
{
    // Private variables
    unsigned char send_buffer[1024];

    // Private variables for received data
    mc_values values;
    int fw_major;
    int fw_minor;
    float rotor_pos;
    mc_configuration mcconf;
    app_configuration appconf;
    float detect_cycle_int_limit;
    float detect_coupling_k;
    signed char detect_hall_table[8];
    signed char detect_hall_res;
    float dec_ppm;
    float dec_ppm_len;
    float dec_adc;
    float dec_adc_voltage;
    float dec_chuk;

    // Function pointers
    void(*send_func)(unsigned char *data, unsigned int len);
    void(*forward_func)(unsigned char *data, unsigned int len);

    // Function pointers for received data
    void(*rx_value_func)(mc_values *values);
    void(*rx_printf_func)(char *str);
    void(*rx_fw_func)(int major, int minor);
    void(*rx_rotor_pos_func)(float pos);
    void(*rx_mcconf_func)(mc_configuration *conf);
    void(*rx_appconf_func)(app_configuration *conf);
    void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
            const signed char *hall_table, signed char hall_res);
    void(*rx_dec_ppm_func)(float val, float ms);
    void(*rx_dec_adc_func)(float val, float voltage);
    void(*rx_dec_chuk_func)(float val);
    void(*rx_mcconf_received_func)(void);
    void(*rx_appconf_received_func)(void);
    void(*motor_control_set_func)(motor_control_mode mode, float value);
    void(*values_requested_func)(void);
} BldcInterface;

// interface functions
void bldc_interface_init(BldcInterface* interface, void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_set_forward_func(BldcInterface* interface, void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_send_packet(BldcInterface* interface, unsigned char *data, unsigned int len);
void bldc_interface_process_packet(BldcInterface* interface, unsigned char *data, unsigned int len);

// Function pointer setters
void bldc_interface_set_rx_value_func(BldcInterface* interface, void(*func)(mc_values *values));
void bldc_interface_set_rx_printf_func(BldcInterface* interface, void(*func)(char *str));
void bldc_interface_set_rx_fw_func(BldcInterface* interface, void(*func)(int major, int minor));
void bldc_interface_set_rx_rotor_pos_func(BldcInterface* interface, void(*func)(float pos));
void bldc_interface_set_rx_mcconf_func(BldcInterface* interface, void(*func)(mc_configuration *conf));
void bldc_interface_set_rx_appconf_func(BldcInterface* interface, void(*func)(app_configuration *conf));
void bldc_interface_set_rx_detect_func(BldcInterface* interface, void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res));
void bldc_interface_set_rx_dec_ppm_func(BldcInterface* interface, void(*func)(float val, float ms));
void bldc_interface_set_rx_dec_adc_func(BldcInterface* interface, void(*func)(float val, float voltage));
void bldc_interface_set_rx_dec_chuk_func(BldcInterface* interface, void(*func)(float val));
void bldc_interface_set_rx_mcconf_received_func(BldcInterface* interface, void(*func)(void));
void bldc_interface_set_rx_appconf_received_func(BldcInterface* interface, void(*func)(void));

void bldc_interface_set_sim_control_function(BldcInterface* interface, void(*func)(motor_control_mode mode, float value));
void bldc_interface_set_sim_values_func(BldcInterface* interface, void(*func)(void));

// Setters
void bldc_interface_terminal_cmd(BldcInterface* interface, char* cmd);
void bldc_interface_set_duty_cycle(BldcInterface* interface, float dutyCycle);
void bldc_interface_set_current(BldcInterface* interface, float current);
void bldc_interface_set_current_brake(BldcInterface* interface, float current);
void bldc_interface_set_rpm(BldcInterface* interface, int rpm);
void bldc_interface_set_pos(BldcInterface* interface, float pos);
void bldc_interface_set_handbrake(BldcInterface* interface, float current);
void bldc_interface_set_servo_pos(BldcInterface* interface, float pos);
void bldc_interface_set_mcconf(BldcInterface* interface, const mc_configuration *mcconf);
void bldc_interface_set_appconf(BldcInterface* interface, const app_configuration *appconf);

// Getters
void bldc_interface_get_fw_version(BldcInterface* interface);
void bldc_interface_get_values(BldcInterface* interface);
void bldc_interface_get_mcconf(BldcInterface* interface);
void bldc_interface_get_appconf(BldcInterface* interface);
void bldc_interface_get_decoded_ppm(BldcInterface* interface);
void bldc_interface_get_decoded_adc(BldcInterface* interface);
void bldc_interface_get_decoded_chuk(BldcInterface* interface);

// Other functions
void bldc_interface_detect_motor_param(BldcInterface*, float current, float min_rpm, float low_duty);
void bldc_interface_reboot(BldcInterface*);
void bldc_interface_send_alive(BldcInterface*);
void send_values_to_receiver(BldcInterface*, mc_values *values);

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault);

#endif /* BLDC_INTERFACE_H_ */
