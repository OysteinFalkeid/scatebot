
#ifndef MOTOR_DEFINES
#define MOTOR_DEFINES

#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <Arduino.h>

constexpr uint32_t BAUD_300 = 300;
constexpr uint32_t BAUD_600 = 600;
constexpr uint32_t BAUD_750 = 750;
constexpr uint32_t BAUD_1200 = 1200;
constexpr uint32_t BAUD_2400 = 2400;
constexpr uint32_t BAUD_4800 = 4800;
constexpr uint32_t BAUD_9600 = 9600;
constexpr uint32_t BAUD_19200 = 19200;
constexpr uint32_t BAUD_31250 = 31250;
constexpr uint32_t BAUD_38400 = 38400;
constexpr uint32_t BAUD_57600 = 57600;
constexpr uint32_t BAUD_74880 = 74880;
constexpr uint32_t BAUD_115200 = 115200;

constexpr uint32_t BAUD = BAUD_115200;
constexpr uint32_t BAUD_BOOT = BAUD_115200;

constexpr bool FORWARD = true;
constexpr bool REVERSE = false;

constexpr bool START = true;
constexpr bool STOP = false;

constexpr uint32_t INTR_LOW_PRI = 0;

constexpr uint32_t MCPWM_0 = 0;
constexpr uint32_t MCPWM_1 = 1;

// Define GPIO pins for BLDC motor control
#define PWM_0_AH_PIN    GPIO_NUM_21  // Phase A High
#define PWM_0_AL_PIN    GPIO_NUM_4  // Phase A Low
#define PWM_0_BH_PIN    GPIO_NUM_5  // Phase B High  
#define PWM_0_BL_PIN    GPIO_NUM_12  // Phase B Low
#define PWM_0_CH_PIN    GPIO_NUM_13  // Phase C High
#define PWM_0_CL_PIN    GPIO_NUM_14  // Phase C Low

// Hall sensor pins (analog inputs)
#define HALL_0A_PIN    GPIO_NUM_26  // GPIO36
#define HALL_0B_PIN    GPIO_NUM_27  // GPIO39
#define HALL_0C_PIN    GPIO_NUM_32  // GPIO34

// Define GPIO pins for BLDC motor control
#define PWM_1_AH_PIN    GPIO_NUM_15  // Phase A High
#define PWM_1_AL_PIN    GPIO_NUM_16  // Phase A Low
#define PWM_1_BH_PIN    GPIO_NUM_17  // Phase B High  
#define PWM_1_BL_PIN    GPIO_NUM_18  // Phase B Low
#define PWM_1_CH_PIN    GPIO_NUM_19  // Phase C High
#define PWM_1_CL_PIN    GPIO_NUM_25  // Phase C Low

// Hall sensor pins (analog inputs)
#define HALL_1A_PIN    GPIO_NUM_33  // GPIO36
#define HALL_1B_PIN    GPIO_NUM_34  // GPIO39
#define HALL_1C_PIN    GPIO_NUM_35  // GPIO34

// BLDC control parameters
constexpr uint32_t PWM_FREQUENCY = 20000;    // 20kHz PWM frequency
constexpr uint32_t TIMER_RESOLUTION = 160000000;  // 1MHz timer resolution (1us per tick)
constexpr uint32_t TIMER_PERIOD_TICK = (TIMER_RESOLUTION / PWM_FREQUENCY);
constexpr uint32_t DEAD_TIME_TICKS = 0;
constexpr uint32_t DUTY_CYCLE_MAX_PROSENT = 2;      // Maximum duty cycle (5%)
constexpr uint32_t SINE_TABLE_SIZE = 3600;    // Sine lookup table size (1 degree resolution)
constexpr uint32_t DUTY_CYCLE_MAX_VALUE = TIMER_PERIOD_TICK * 3 / 100;

constexpr uint32_t NUMBER_OF_STEPS_PER_ROTATION = 30;

constexpr uint32_t HALL_SENSE_PATTERN_0 = (0 << 2) | (1 << 1) | (1 << 0);
constexpr uint32_t HALL_SENSE_PATTERN_1 = (0 << 2) | (1 << 1) | (0 << 0);
constexpr uint32_t HALL_SENSE_PATTERN_2 = (1 << 2) | (1 << 1) | (0 << 0);
constexpr uint32_t HALL_SENSE_PATTERN_3 = (1 << 2) | (0 << 1) | (0 << 0);
constexpr uint32_t HALL_SENSE_PATTERN_4 = (1 << 2) | (0 << 1) | (1 << 0);
constexpr uint32_t HALL_SENSE_PATTERN_5 = (0 << 2) | (0 << 1) | (1 << 0);
constexpr uint32_t HALL_SENSE_DESIMAL_ARRAY[6] = {1, 3, 2, 6, 4, 5};
constexpr uint32_t HALL_SENSE_DESIMAL_TO_POSITION[8] = {8, 0, 2, 1, 4, 5, 3, 8};



#endif