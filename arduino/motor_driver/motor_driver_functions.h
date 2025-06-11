
#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <Arduino.h>

#include "defines.h"
#include "motor_driver_setup.h"

#ifndef MOTOR_FUNCTIONS
#define MOTOR_FUNCTIONS

void set_motor_comutation(uint32_t angle, mcpwm_cmpr_handle_t mcpwm_cmpr_handles[6]);

#endif