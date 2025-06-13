
#include "motor_driver_setup.h"

esp_err_t setup_MCPWM(
    uint32_t                groop,
    mcpwm_timer_handle_t    mcpwm_timer_handles[3],
    mcpwm_oper_handle_t     mcpwm_oper_handles[3],
    mcpwm_cmpr_handle_t     mcpwm_cmpr_handles[6],
    mcpwm_gen_handle_t      mcpwm_gen_handles[6],
    gpio_num_t              motor_gpio_pins[6]) {

    esp_err_t ret = ESP_OK;
    Serial.println("Configuring MCPWM for 3-phase AC generation");
    Serial.printf("MCPWM groop: %d\n", groop);
    mcpwm_timer_config_t mcpwm_timer_config = {
        .group_id       = groop,
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz  = TIMER_RESOLUTION,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks   = TIMER_PERIOD_TICK,
        .intr_priority  = INTR_LOW_PRI
    };

    mcpwm_operator_config_t   mcpwm_operator_config = {
        .group_id       = groop,
        .intr_priority  = INTR_LOW_PRI
    };

    mcpwm_comparator_config_t comparator_config = {
        .intr_priority = INTR_LOW_PRI,
        .flags = {
            .update_cmp_on_tez = true
        }
    };

    mcpwm_generator_config_t  mcpwm_generator_config = {
        .gen_gpio_num = -1,
        .flags = {
            .pull_down = true
        }
    };

    mcpwm_dead_time_config_t  mcpwm_dt_config = {
        .posedge_delay_ticks = DEAD_TIME_TICKS,
        .negedge_delay_ticks = DEAD_TIME_TICKS
    };

    Serial.println("all config variables defines");

    // Create 3 timers for 3-phase control
    for (int i = 0; i < 3; i++) {
        ret = mcpwm_new_timer(&mcpwm_timer_config, &mcpwm_timer_handles[i]);
        if (ret != ESP_OK) {
            Serial.printf("Failed to create timer %d: %s\n", i, esp_err_to_name(ret));
            return ret;
        }
    }

    // Create 3 operators
    for (int i = 0; i < 3; i++) {
        ret = mcpwm_new_operator(&mcpwm_operator_config, &mcpwm_oper_handles[i]);
        if (ret != ESP_OK) {
            Serial.printf("Failed to create operator %d: %s\n", i, esp_err_to_name(ret));
            return ret;
        }
        
        // Connect operator to timer
        ret = mcpwm_operator_connect_timer(mcpwm_oper_handles[i], mcpwm_timer_handles[i]);
        if (ret != ESP_OK) {
            Serial.printf("Failed to connect operator %d to timer: %s\n", i, esp_err_to_name(ret));
            return ret;
        }
    }

    // Create 2 comparators per operator (6 total)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            int idx = i * 2 + j;
            ret = mcpwm_new_comparator(mcpwm_oper_handles[i], &comparator_config, &mcpwm_cmpr_handles[idx]);
            if (ret != ESP_OK) {
                Serial.printf("Failed to create comparator %d: %s\n", idx, esp_err_to_name(ret));
                return ret;
            }
            
            // Set initial compare value to 0
            mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[idx], 0);
        }
    }

    // Create generators and configure GPIO
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            int idx = i * 2 + j;
            mcpwm_generator_config.gen_gpio_num = motor_gpio_pins[idx];
            
            ret = mcpwm_new_generator(
                mcpwm_oper_handles[i], 
                &mcpwm_generator_config, 
                &mcpwm_gen_handles[idx]
            );
            if (ret != ESP_OK) {
                Serial.printf("Failed to create generator %d: %s\n", idx, esp_err_to_name(ret));
                return ret;
            }
        }
    }

    // Configure generator actions for Simultainius PWM
    for (int i = 0; i < 6; i++) {
        int idx = i;
        
        ret = mcpwm_generator_set_action_on_timer_event(
            mcpwm_gen_handles[idx],
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,   // direction to count after the event
                MCPWM_TIMER_EVENT_EMPTY,    // When dose the event occur
                MCPWM_GEN_ACTION_HIGH       // What action to do
            )
        );
        if (ret != ESP_OK) {
            Serial.printf("Failed to set generator action on MCPWM timer event %d: %s\n", idx, esp_err_to_name(ret));
            return ret;
        }

        ret = mcpwm_generator_set_action_on_compare_event(
            mcpwm_gen_handles[idx],
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,   // direction to count after the event
                mcpwm_cmpr_handles[idx],    // When dose the event occur
                MCPWM_GEN_ACTION_LOW        // What action to do
            )
        );
        if (ret != ESP_OK) {
            Serial.printf("Failed to set generator action on MCPWM timer comp event %d: %s\n", idx, esp_err_to_name(ret));
            return ret;
        }
    }
    
    if (DEAD_TIME_TICKS > 0) {
        for (int i = 0; i < 3; i++) {
            int high_idx = i * 2;
            int low_idx = i * 2 + 1;
            
            ret = mcpwm_generator_set_dead_time(
                mcpwm_gen_handles[high_idx], 
                mcpwm_gen_handles[low_idx], 
                &mcpwm_dt_config
            );
            if (ret != ESP_OK) {
                Serial.printf("Failed to set dead time for phase %d: %s\n", i, esp_err_to_name(ret));
                return ret;
            }
        }
    }

    // Enable all timers
    for (int i = 0; i < 3; i++) {
        ret = mcpwm_timer_enable(mcpwm_timer_handles[i]);
        if (ret != ESP_OK) {
            Serial.printf("Failed to enable timer %d: %s\n", i, esp_err_to_name(ret));
            return ret;
        }
        
        ret = mcpwm_timer_start_stop(mcpwm_timer_handles[i], MCPWM_TIMER_START_NO_STOP);
        if (ret != ESP_OK) {
            Serial.printf("Failed to start timer %d: %s\n", i, esp_err_to_name(ret));
            return ret;
        }
    }

    return ret;
}


void Initialize_sine_table(uint32_t* sine_table) {
  // Pre-calculate sine values for smooth AC generation
  for (uint32_t i = 0; i < SINE_TABLE_SIZE; i++) {
    sine_table[i] = (uint32_t) sin(i * (PI / (SINE_TABLE_SIZE / 2)));
  }
  Serial.println("Sine lookup table initialized");
}

esp_err_t Setup_GPIO_pins_output(gpio_num_t* gpio_pins) {
    esp_err_t ret = ESP_OK;
    // Configure all motor 0 pins as outputs
    for (int i = 0; i <= sizeof(gpio_pins); i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask   = (1ULL << gpio_pins[i]),
            .mode           = GPIO_MODE_OUTPUT,
            .pull_up_en     = GPIO_PULLUP_DISABLE,
            .pull_down_en   = GPIO_PULLDOWN_DISABLE,
            .intr_type      = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            Serial,printf("Failed to set otput %d: %s\n", i, esp_err_to_name(ret));
            break;
        }
    }
    return ret;
}

esp_err_t Setup_GPIO_pins_input(gpio_num_t* gpio_pins) {
    esp_err_t ret = ESP_OK;
    // Configure all motor 0 pins as outputs
    for (int i = 0; i <= sizeof(gpio_pins); i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask   = (1ULL << gpio_pins[i]),
            .mode           = GPIO_MODE_INPUT,
            .pull_up_en     = GPIO_PULLUP_ENABLE,
            .pull_down_en   = GPIO_PULLDOWN_DISABLE,
            .intr_type      = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            Serial,printf("Failed to set otput %d: %s\n", i, esp_err_to_name(ret));
            break;
        }
    }
    return ret;
}

void Setup_seriel_at_boot(uint32_t baud) {
    delay(100);
    Serial.begin(BAUD_BOOT);
    delay(100);
    Serial.printf("BAUD%d\n", baud);
    delay(100);
    Serial.begin(baud);
    delay(100);
    Serial.println("ESP32 3-Phase BLDC Motor Driver Initializing...");
}




