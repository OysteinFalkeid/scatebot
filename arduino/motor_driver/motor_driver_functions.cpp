#include "defines.h"
#include "motor_driver_setup.h"
#include "motor_driver_functions.h"

esp_err_t Set_compare_value(mcpwm_cmpr_handle_t mcpwm_cmpr_handles[6], uint32_t angle){
    esp_err_t ret = ESP_OK;
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[0], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[1], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[2], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[3], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[4], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }
  
    ret = mcpwm_comparator_set_compare_value(mcpwm_cmpr_handles[5], angle);
    if (ret != ESP_OK) {
      Serial.printf("Failed to set compare %s\n", esp_err_to_name(ret));
    }

    return ret;
}
