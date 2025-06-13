#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <Arduino.h>

#include "defines.h"

#ifndef MOTOR_SETUP
#define MOTOR_SETUP


  esp_err_t setup_MCPWM(
    uint32_t                groop,
    mcpwm_timer_handle_t    mcpwm_timer_handles_0[3],
    mcpwm_oper_handle_t     mcpwm_oper_handles_0[3],
    mcpwm_cmpr_handle_t     mcpwm_cmpr_handles_0[6],
    mcpwm_gen_handle_t      mcpwm_gen_handles_0[6],
    gpio_num_t              motor_gpio_pins[6]
);

void Initialize_sine_table(uint32_t* sine_table);

esp_err_t Setup_GPIO_pins_output(gpio_num_t* gpio_pins);
esp_err_t Setup_GPIO_pins_input(gpio_num_t* gpio_pins);

void Setup_seriel_at_boot(uint32_t baud);

#endif