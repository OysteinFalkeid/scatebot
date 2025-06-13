#include "motor_driver.h"

// MCPWM 0 handles and configuration
gpio_num_t motor_0_gpio_pins[6]  = {PWM_0_AH_PIN, PWM_0_AL_PIN, PWM_0_BH_PIN, PWM_0_BL_PIN, PWM_0_CH_PIN, PWM_0_CL_PIN};

// Motor 0 control variables
uint32_t motor_0_speed = 0;     // Motor speed (0-10000)
bool motor_0_direction = FORWARD; // Motor direction (true = forward)
bool motor_0_enabled = STOP;  // Motor enable state
uint32_t motor_0_electrical_angle = 0; // Current electrical angle (0-36000 degrees)
uint32_t motor_0_angle_increment = 1;  // Angle increment per update

// Hall sensor variables
bool motor_0_hall_values[3] = {false, false, false};
uint32_t motor_0_hall_angle_offset = 0; // Angle offset from hall sensors
gpio_num_t hall_0_sense_pins[3] = {HALL_0A_PIN, HALL_0B_PIN, HALL_0C_PIN};


// MCPWM 1 handles and configuration
gpio_num_t motor_1_gpio_pins[6]  = {PWM_1_AH_PIN, PWM_1_AL_PIN, PWM_1_BH_PIN, PWM_1_BL_PIN, PWM_1_CH_PIN, PWM_1_CL_PIN};

// Motor 1 control variables
uint32_t motor_1_speed = 0;     // Motor speed (0-10000)
bool motor_1_direction = FORWARD; // Motor direction (true = forward)
bool motor_1_enabled = STOP;  // Motor enable state
uint32_t motor_1_electrical_angle = 0; // Current electrical angle (0-36000 degrees)
uint32_t motor_1_angle_increment = 1;  // Angle increment per update

// Hall sensor variables
bool motor_1_hall_values[3] = {false, false, false};
uint32_t motor_1_hall_angle_offset = 0; // Angle offset from hall sensors
gpio_num_t hall_1_sense_pins[3] = {HALL_1A_PIN, HALL_1B_PIN, HALL_1C_PIN};

void setup() {
  // put your setup code here, to run once:
  esp_err_t error = ESP_OK;

  Setup_seriel_at_boot(BAUD);

  Initialize_sine_table(sine_table);

  error = Setup_GPIO_pins_output(motor_0_gpio_pins);
  if (error != ESP_OK) {
    Serial.println("Failed initialise pins for motor 0");
    esp_restart();
  }
  error = Setup_GPIO_pins_output(motor_1_gpio_pins);
  if (error != ESP_OK) {
    Serial.println("Failed initialise pins for motor 1");
    esp_restart();
  }  
  
  error = Setup_GPIO_pins_input(hall_0_sense_pins);
  if (error != ESP_OK) {
    Serial.println("Failed initialise pins for hall 0");
    esp_restart();
  }
  error = Setup_GPIO_pins_input(hall_1_sense_pins);
  if (error != ESP_OK) {
    Serial.println("Failed initialise pins for hall 1");
    esp_restart();
  }

  error = setup_MCPWM(
    0,
    mcpwm_timer_handles_0,
    mcpwm_oper_handles_0,
    mcpwm_cmpr_handles_0,
    mcpwm_gen_handles_0,
    motor_0_gpio_pins
  );

  if (error == ESP_OK) {
    Serial.println("MCPWM 0 initialized successfully");
  } else {
    Serial.println("MCPWM 0 initialization failed!");
    esp_restart();
  }

  error = setup_MCPWM(
    1,
    mcpwm_timer_handles_1,
    mcpwm_oper_handles_1,
    mcpwm_cmpr_handles_1,
    mcpwm_gen_handles_1,
    motor_1_gpio_pins
  );

  if (error == ESP_OK) {
    Serial.println("MCPWM 0 initialized successfully");
  } else {
    Serial.println("MCPWM 0 initialization failed!");
    esp_restart();
  }
}


void loop() {
  static uint32_t global_counter;
  // put your main code here, to run repeatedly:
  esp_err_t error = ESP_OK;
  global_counter += 1;
  if (global_counter > 1000) {
    global_counter = 1;
  }

  delayMicroseconds(6000);

  uint32_t duty = (uint32_t) DUTY_CYCLE_MAX_VALUE * global_counter / 1000;

  error = Set_compare_value(mcpwm_cmpr_handles_0, duty);
  if (error != ESP_OK) {
    Serial.println("Failed to set comare value");
  }

  error = Set_compare_value(mcpwm_cmpr_handles_1, duty);
  if (error != ESP_OK) {
    Serial.println("Failed to set comare value");
  }
}