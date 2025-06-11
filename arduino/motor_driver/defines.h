
#ifndef MOTOR_DEFINES
#define MOTOR_DEFINES

constexpr int BAUD_300 = 300;
#define BAUD_600 600
#define BAUD_750 750
#define BAUD_1200 1200
#define BAUD_2400 2400
#define BAUD_4800 4800
#define BAUD_9600 9600
#define BAUD_19200 19200
#define BAUD_31250 31250
#define BAUD_38400 38400
#define BAUD_57600 57600
#define BAUD_74880 74880
#define BAUD_115200 115200

#define BAUD BAUD_115200
#define BAUD_BOOT BAUD_115200

#define FORWARD true
#define REVERSE false

#define START true
#define STOP false

#define INTR_LOW_PRI 0

#define MCPWM_0 0
#define MCPWM_1 1

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
#define PWM_FREQUENCY     20000    // 20kHz PWM frequency
#define TIMER_RESOLUTION  1000000  // 1MHz timer resolution (1us per tick)
#define TIMER_PERIOD_TICK  (TIMER_RESOLUTION / PWM_FREQUENCY)
#define DEAD_TIME_TICKS   0
#define DUTY_CYCLE_MAX    5      // Maximum duty cycle (5%)
#define SINE_TABLE_SIZE   360    // Sine lookup table size (1 degree resolution)



#endif