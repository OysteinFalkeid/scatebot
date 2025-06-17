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
uint8_t hall_0_sense_pins_pattern[6] = {
  HALL_SENSE_PATTERN_0,
  HALL_SENSE_PATTERN_1,
  HALL_SENSE_PATTERN_2,
  HALL_SENSE_PATTERN_3,
  HALL_SENSE_PATTERN_4,
  HALL_SENSE_PATTERN_5
};


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

uint32_t global_counter = 0;
uint32_t counter_right_wheel = 0;
uint32_t right_wheel_sens_position = 0;

esp_timer_handle_t motor_control_timer_handel;

void motor_control_callback(void* args) {
  // Very relaxed timing requirements at low speeds
  // electrical_angle += angle_increment;
  // if (electrical_angle >= 360.0f) electrical_angle -= 360.0f;
  
  // Update PWM duty cycles
  // update_sine_pwm_duties(electrical_angle);

  uint8_t hall_00_sense_pin = digitalRead(hall_0_sense_pins[0]);
  uint8_t hall_01_sense_pin = digitalRead(hall_0_sense_pins[1]);
  uint8_t hall_02_sense_pin = digitalRead(hall_0_sense_pins[2]);

  uint8_t hall_0_sense_pin = (hall_00_sense_pin << 0) | (hall_01_sense_pin << 1) | (hall_02_sense_pin << 2);
  Serial.printf("H%c\n", HALL_SENSE_DESIMAL_TO_POSITION[hall_0_sense_pin]);
}





void setup() {
  // put your setup code here, to run once:
  esp_err_t error = ESP_OK;

  Setup_seriel_at_boot(BAUD);

  Initialize_sine_table(sine_table);

  // pinMode();

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

  // error = Setup_motor_timer_calback(motor_control_callback, motor_control_timer_handel, MICROS_BETWEN_TIMER_INTERUPTS);  
  // if (error != ESP_OK)
  //   Serial.println("Failed to start general timer");
  
  esp_timer_create_args_t timer_args = {
    .callback = motor_control_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "motor_ctrl",
    .skip_unhandled_events = true
  };

  error = esp_timer_create(&timer_args, &motor_control_timer_handel);
  if (error != ESP_OK)
      Serial.printf("Failed to initialize general timer for consistant interupts for motor control: %s\n", esp_err_to_name(error));
      
  error = esp_timer_start_periodic(motor_control_timer_handel, MICROS_BETWEN_TIMER_INTERUPTS);
  if (error != ESP_OK)
      Serial.printf("Failed to start general timer periode time: %d, %s\n", micros, esp_err_to_name(error));
    


  if (error == ESP_OK) {
    Serial.println("setup successfully");
  } else {
    Serial.println("setup failed!");
    esp_restart();
  }

}


void loop() {
  // put your main code here, to run repeatedly:
  esp_err_t error = ESP_OK;
  global_counter += 1;
  if (global_counter > 1000) {
    global_counter = 1;
  }

  // if (!(global_counter % 10)) {
  //   uint8_t hall_00_sense_pin = digitalRead(hall_0_sense_pins[0]);
  //   uint8_t hall_01_sense_pin = digitalRead(hall_0_sense_pins[1]);
  //   uint8_t hall_02_sense_pin = digitalRead(hall_0_sense_pins[2]);
  
  //   uint8_t hall_0_sense_pin = (hall_00_sense_pin << 0) | (hall_01_sense_pin << 1) | (hall_02_sense_pin << 2);
  //   Serial.printf("H%c\n", HALL_SENSE_DESIMAL_TO_POSITION[hall_0_sense_pin]);
  //   // uint32_t right_wheel_sens_position_new = HALL_SENSE_DESIMAL_TO_POSITION[hall_0_sense_pin];

  //   // if (right_wheel_sens_position_new = 0) {
  //   //   if (right_wheel_sens_position = 5) {
  //   //     // rotation forward
  //   //   } else if (right_wheel_sens_position = 1) {
  //   //     // rotation reverse
  //   //   }
  //   // } else if (right_wheel_sens_position > right_wheel_sens_position_new){

  //   // }
  // }

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