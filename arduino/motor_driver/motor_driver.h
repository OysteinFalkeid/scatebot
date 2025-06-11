
#ifndef MOTOR
#define MOTOR

#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <Arduino.h>

#include "defines.h"

// functions uppdating global variables

void initializeSineTable(float* sine_table);

esp_err_t initializeMCPWM(void);

void testGPIOPins(void);

void setup(void);

void loop(void);

#endif