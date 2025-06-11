#include "motor_driver.h"

// MCPWM 0 handles and configuration
mcpwm_timer_config_t      mcpwm_timer_config_0;
mcpwm_timer_handle_t      mcpwm_timer_handles_0[3];
mcpwm_operator_config_t   mcpwm_operator_config_0;
mcpwm_oper_handle_t       mcpwm_oper_handles_0[3];
mcpwm_comparator_config_t comparator_config_0;
mcpwm_cmpr_handle_t       mcpwm_cmpr_handles_0[6]; // 2 comparators per operator
mcpwm_generator_config_t  mcpwm_generator_config_0;
gpio_num_t motor_0_gpio_pins[6]  = {PWM_0_AH_PIN, PWM_0_AL_PIN, PWM_0_BH_PIN, PWM_0_BL_PIN, PWM_0_CH_PIN, PWM_0_CL_PIN};
mcpwm_gen_handle_t        mcpwm_gen_handles_0[6];   // 2 generators per operator
mcpwm_dead_time_config_t  mcpwm_dt_config_0;

// Motor 0 control variables
uint32_t motor_0_speed = 0;     // Motor speed (0-10000)
bool motor_0_direction = FORWARD; // Motor direction (true = forward)
bool motor_0_enabled = STOP;  // Motor enable state
uint32_t motor_0_electrical_angle = 0; // Current electrical angle (0-36000 degrees)
uint32_t motor_0_angle_increment = 1;  // Angle increment per update

// Hall sensor variables
bool motor_0_hall_values[3] = {false, false, false};
uint32_t motor_0_hall_angle_offset = 0; // Angle offset from hall sensors

// # endregion

// MCPWM 1 handles and configuration
mcpwm_timer_config_t      mcpwm_timer_config_1;
mcpwm_timer_handle_t      mcpwm_timer_handles_1[3];
mcpwm_operator_config_t   mcpwm_operator_config_1;
mcpwm_oper_handle_t       mcpwm_oper_handles_1[3];
mcpwm_comparator_config_t comparator_config_1;
mcpwm_cmpr_handle_t       mcpwm_cmpr_handles_1[6]; // 2 comparators per operator
mcpwm_generator_config_t  mcpwm_generator_config_1;
gpio_num_t motor_1_gpio_pins[6]  = {PWM_1_AH_PIN, PWM_1_AL_PIN, PWM_1_BH_PIN, PWM_1_BL_PIN, PWM_1_CH_PIN, PWM_1_CL_PIN};
mcpwm_gen_handle_t        mcpwm_gen_handles_1[6];   // 2 generators per operator
mcpwm_dead_time_config_t  mcpwm_dt_config_1;

// Motor 1 control variables
uint32_t motor_1_speed = 0;     // Motor speed (0-10000)
bool motor_1_direction = FORWARD; // Motor direction (true = forward)
bool motor_1_enabled = STOP;  // Motor enable state
uint32_t motor_1_electrical_angle = 0; // Current electrical angle (0-36000 degrees)
uint32_t motor_1_angle_increment = 1;  // Angle increment per update

// Hall sensor variables
bool motor_1_hall_values[3] = {false, false, false};
uint32_t motor_1_hall_angle_offset = 0; // Angle offset from hall sensors

// universal variables

// Sine lookup table for smooth AC generation
float sine_table[SINE_TABLE_SIZE];

// functions uppdating global variables

void initializeSineTable(float* sine_table) {
  // Pre-calculate sine values for smooth AC generation
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    sine_table[i] = sin(i * PI / 180.0);
  }
  Serial.println("Sine lookup table initialized");
}

esp_err_t initializeMCPWM(void) {
  esp_err_t ret = ESP_OK;

  Serial.println("Configuring MCPWM for 3-phase AC generation...");
  Serial.printf("Timer period ticks: %d\n", TIMER_PERIOD_TICK);

  // Initialize configuration structures
  memset(&mcpwm_timer_config_0, 0, sizeof(mcpwm_timer_config_0));
  memset(&mcpwm_operator_config_0, 0, sizeof(mcpwm_operator_config_0));
  memset(&comparator_config_0, 0, sizeof(comparator_config_0));
  memset(&mcpwm_generator_config_0, 0, sizeof(mcpwm_generator_config_0));
  memset(&mcpwm_dt_config_0, 0, sizeof(mcpwm_dt_config_0));

  // defining the MCPWM peripheral timers config
  mcpwm_timer_config_0.group_id = MCPWM_0;
  mcpwm_timer_config_0.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  mcpwm_timer_config_0.resolution_hz = TIMER_RESOLUTION;
  mcpwm_timer_config_0.period_ticks = TIMER_PERIOD_TICK;
  mcpwm_timer_config_0.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  mcpwm_timer_config_0.intr_priority = INTR_LOW_PRI;


  // Create 3 timers for 3-phase control
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_new_timer(&mcpwm_timer_config_0, &mcpwm_timer_handles_0[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to create timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }

  mcpwm_operator_config_0.group_id = MCPWM_0;
  mcpwm_operator_config_0.intr_priority = INTR_LOW_PRI;

  // Create 3 operators
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_new_operator(&mcpwm_operator_config_0, &mcpwm_oper_handles_0[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to create operator %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
    
    // Connect operator to timer
    ret = mcpwm_operator_connect_timer(mcpwm_oper_handles_0[i], mcpwm_timer_handles_0[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to connect operator %d to timer: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }

  comparator_config_0.intr_priority = INTR_LOW_PRI;
  comparator_config_0.flags.update_cmp_on_tez = true;

  // Create 2 comparators per operator (6 total)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      ret = mcpwm_new_comparator(mcpwm_oper_handles_0[i], &comparator_config_0, &mcpwm_cmpr_handles_0[idx]);
      if (ret != ESP_OK) {
        Serial.printf("Failed to create comparator %d: %s\n", idx, esp_err_to_name(ret));
        return ret;
      }
      
      // Set initial compare value to 0
      mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[idx], 0);
    }
  }

  mcpwm_generator_config_0.gen_gpio_num = -1;
  mcpwm_generator_config_0.flags.pull_down = true;

  // Create generators and configure GPIO
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      mcpwm_generator_config_0.gen_gpio_num = motor_0_gpio_pins[idx];
      
      ret = mcpwm_new_generator(mcpwm_oper_handles_0[i], &mcpwm_generator_config_0, &mcpwm_gen_handles_0[idx]);
      if (ret != ESP_OK) {
        Serial.printf("Failed to create generator %d: %s\n", idx, esp_err_to_name(ret));
        return ret;
      }
    }
  }

  // Configure generator actions for complementary PWM
  // TODO change this as the mosfet driver creates the complimentary PWM from identical PWM because of inverted low mosfet
  for (int i = 0; i < 3; i++) {
    int high_idx = i * 2;     // High side index
    int low_idx = i * 2 + 1;  // Low side index
    
    // High side generator actions
    mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handles_0[high_idx],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handles_0[high_idx],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmpr_handles_0[high_idx], MCPWM_GEN_ACTION_LOW));
    
    // Low side generator actions (complementary)
    mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handles_0[low_idx],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handles_0[low_idx],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmpr_handles_0[low_idx], MCPWM_GEN_ACTION_LOW));
  }
  
  mcpwm_dt_config_0.posedge_delay_ticks = DEAD_TIME_TICKS;
  mcpwm_dt_config_0.negedge_delay_ticks = DEAD_TIME_TICKS;
  
  if (DEAD_TIME_TICKS > 0) {
    for (int i = 0; i < 3; i++) {
      int high_idx = i * 2;
      int low_idx = i * 2 + 1;
      
      ret = mcpwm_generator_set_dead_time(mcpwm_gen_handles_0[high_idx], mcpwm_gen_handles_0[low_idx], &mcpwm_dt_config_0);
      if (ret != ESP_OK) {
        Serial.printf("Failed to set dead time for phase %d: %s\n", i, esp_err_to_name(ret));
        return ret;
      }
    }
  }
    
  // Enable all timers
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_timer_enable(mcpwm_timer_handles_0[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to enable timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
    
    ret = mcpwm_timer_start_stop(mcpwm_timer_handles_0[i], MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
      Serial.printf("Failed to start timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }

  // Initialize configuration structures
  memset(&mcpwm_timer_config_1, 0, sizeof(mcpwm_timer_config_1));
  memset(&mcpwm_operator_config_1, 0, sizeof(mcpwm_operator_config_1));
  memset(&comparator_config_1, 0, sizeof(comparator_config_1));
  memset(&mcpwm_generator_config_1, 0, sizeof(mcpwm_generator_config_1));
  memset(&mcpwm_dt_config_1, 0, sizeof(mcpwm_dt_config_1));

  mcpwm_timer_config_1.group_id = MCPWM_1;
  mcpwm_timer_config_1.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  mcpwm_timer_config_1.resolution_hz = TIMER_RESOLUTION;
  mcpwm_timer_config_1.period_ticks = TIMER_PERIOD_TICK;
  mcpwm_timer_config_1.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  mcpwm_timer_config_1.intr_priority = INTR_LOW_PRI;


  // Create 3 timers for 3-phase control
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_new_timer(&mcpwm_timer_config_1, &mcpwm_timer_handles_1[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to create timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }

  mcpwm_operator_config_1.group_id = MCPWM_1;
  mcpwm_operator_config_1.intr_priority = INTR_LOW_PRI;

  // Create 3 operators
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_new_operator(&mcpwm_operator_config_1, &mcpwm_oper_handles_1[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to create operator %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
    
    // Connect operator to timer
    ret = mcpwm_operator_connect_timer(mcpwm_oper_handles_1[i], mcpwm_timer_handles_1[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to connect operator %d to timer: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }

  comparator_config_1.intr_priority = INTR_LOW_PRI;
  comparator_config_1.flags.update_cmp_on_tez = true;

  // Create 2 comparators per operator (6 total)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      ret = mcpwm_new_comparator(mcpwm_oper_handles_1[i], &comparator_config_1, &mcpwm_cmpr_handles_1[idx]);
      if (ret != ESP_OK) {
        Serial.printf("Failed to create comparator %d: %s\n", idx, esp_err_to_name(ret));
        return ret;
      }
      
      // Set initial compare value to 0
      mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[idx], 0);
    }
  }

  mcpwm_generator_config_1.gen_gpio_num = -1;
  mcpwm_generator_config_1.flags.pull_down = true;

  // Create generators and configure GPIO
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      mcpwm_generator_config_1.gen_gpio_num = motor_1_gpio_pins[idx];
      
      ret = mcpwm_new_generator(mcpwm_oper_handles_1[i], &mcpwm_generator_config_1, &mcpwm_gen_handles_1[idx]);
      if (ret != ESP_OK) {
        Serial.printf("Failed to create generator %d: %s\n", idx, esp_err_to_name(ret));
        return ret;
      }
    }
  }

  // Configure generator actions for complementary PWM
  // TODO change this as the mosfet driver creates the complimentary PWM from identical PWM because of inverted low mosfet
  for (int i = 0; i < 3; i++) {
    int high_idx = i * 2;     // High side index
    int low_idx = i * 2 + 1;  // Low side index
    
    // High side generator actions
    mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handles_1[high_idx],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handles_1[high_idx],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmpr_handles_1[high_idx], MCPWM_GEN_ACTION_LOW));
    
    // Low side generator actions (complementary)
    mcpwm_generator_set_action_on_timer_event(mcpwm_gen_handles_1[low_idx],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(mcpwm_gen_handles_1[low_idx],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmpr_handles_1[low_idx], MCPWM_GEN_ACTION_LOW));
  }
  
  mcpwm_dt_config_1.posedge_delay_ticks = DEAD_TIME_TICKS;
  mcpwm_dt_config_1.negedge_delay_ticks = DEAD_TIME_TICKS;
  
  for (int i = 0; i < 3; i++) {
    int high_idx = i * 2;
    int low_idx = i * 2 + 1;
    
    ret = mcpwm_generator_set_dead_time(mcpwm_gen_handles_1[high_idx], mcpwm_gen_handles_1[low_idx], &mcpwm_dt_config_1);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set dead time for phase %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }
  
  // Enable all timers
  for (int i = 0; i < 3; i++) {
    ret = mcpwm_timer_enable(mcpwm_timer_handles_1[i]);
    if (ret != ESP_OK) {
      Serial.printf("Failed to enable timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
    
    ret = mcpwm_timer_start_stop(mcpwm_timer_handles_1[i], MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
      Serial.printf("Failed to start timer %d: %s\n", i, esp_err_to_name(ret));
      return ret;
    }
  }


  return ret;
}

void testGPIOPins() {
  // Configure all motor 0 pins as outputs
  for (int i = 0; i < 6; i++) {
    gpio_config_t io_conf_0 = {};
    io_conf_0.intr_type = GPIO_INTR_DISABLE;
    io_conf_0.mode = GPIO_MODE_OUTPUT;
    io_conf_0.pin_bit_mask = (1ULL << motor_0_gpio_pins[i]);
    io_conf_0.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_0.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_0);

    gpio_config_t io_conf_1 = {};
    io_conf_1.intr_type = GPIO_INTR_DISABLE;
    io_conf_1.mode = GPIO_MODE_OUTPUT;
    io_conf_1.pin_bit_mask = (1ULL << motor_1_gpio_pins[i]);
    io_conf_1.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_1.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_1);
  }
}

void setup() {
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(BAUD_BOOT);
  Serial.printf("BAUD%d\n", BAUD);
  Serial.begin(BAUD);
  delay(100);
  Serial.println("ESP32 3-Phase BLDC Motor Driver Initializing...");

  initializeSineTable(sine_table);
  testGPIOPins();

  // Initialize MCPWM
  if (initializeMCPWM() == ESP_OK) {
    Serial.println("MCPWM initialized successfully");
  } else {
    Serial.println("MCPWM initialization failed!");
    esp_restart();
  }

}

uint32_t counter;

void loop() {
  // put your main code here, to run repeatedly:
  counter += 1;
  if (counter > 10000) {
    counter = 1;
  }

  delayMicroseconds(100);
  // Serial.printf("counter %d\n", counter);

  uint32_t duty = (uint32_t) TIMER_PERIOD_TICK * counter / 100000;

  esp_err_t ret = ESP_OK;

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[0], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[1], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[2], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[3], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[4], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_0[5], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }




  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[0], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[1], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[2], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[3], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[4], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

  ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles_1[5], duty);
  if (ret != ESP_OK) {
    Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
  }

}