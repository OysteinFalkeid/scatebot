#include "defines.h"
#include "motor_driver_setup.h"
#include "motor_driver_functions.h"

esp_err_t Set_compare_value(mcpwm_cmpr_handle_t mcpwm_cmpr_handles[6], uint32_t angle, float* sine_table){
    esp_err_t ret = ESP_OK;

    uint32_t duty_0 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[angle%360]+1) / 2);
    uint32_t duty_1 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[(angle+72)%360]+1) / 2);
    uint32_t duty_2 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[(angle+144)%360]+1) / 2);
    uint32_t duty_3 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[(angle+216)%360]+1) / 2);
    uint32_t duty_4 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[(angle+288)%360]+1) / 2);
    uint32_t duty_5 = (uint32_t) (DUTY_CYCLE_MAX_VALUE * (sine_table[(angle+360)%360]+1) / 2);
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[0], duty_0);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[1], duty_1);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[2], duty_2);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[3], duty_3);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[4], duty_4);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[5], duty_5);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }

    return ret;
}
