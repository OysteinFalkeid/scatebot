#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <Arduino.h>

#include "defines.h"
#include "motor_driver_setup.h"
#include "motor_driver_functions.h"

#ifndef MOTOR
#define MOTOR

#define F_CPU 240000000

// MCPWM 0 handles and configuration
mcpwm_timer_handle_t      mcpwm_timer_handles_0[3];
mcpwm_oper_handle_t       mcpwm_oper_handles_0[3];
mcpwm_cmpr_handle_t       mcpwm_cmpr_handles_0[6]; // 2 comparators per operator
mcpwm_gen_handle_t        mcpwm_gen_handles_0[6];   // 2 generators per operator


// MCPWM 1 handles and configuration
mcpwm_timer_handle_t      mcpwm_timer_handles_1[3];
mcpwm_oper_handle_t       mcpwm_oper_handles_1[3];
mcpwm_cmpr_handle_t       mcpwm_cmpr_handles_1[6]; // 2 comparators per operator
mcpwm_gen_handle_t        mcpwm_gen_handles_1[6];   // 2 generators per operator


// Sine lookup table for smooth AC generation
uint32_t sine_table[SINE_TABLE_SIZE];

uint32_t hall_sense_pattern[6] = {HALL_SENSE_PATTERN_0, HALL_SENSE_PATTERN_1, HALL_SENSE_PATTERN_2, HALL_SENSE_PATTERN_3, HALL_SENSE_PATTERN_4, HALL_SENSE_PATTERN_5};


void setup(void);

void loop(void);

#endif