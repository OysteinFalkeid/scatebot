

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "defines.h"
#include "motor_driver_setup.h"

#ifndef MOTOR_FUNCTIONS
#define MOTOR_FUNCTIONS

// is sinusodial waveform is wanted this is an array form 0 to pi/2 with 255 steps.
// created by gpt
// TODO manualy calculate to ensure corect table values
#include <avr/pgmspace.h>
extern const uint8_t sine_table[128];


// Global variables
extern volatile uint8_t prescaler;
extern volatile uint8_t motor0_commutation[8];
extern volatile uint8_t motor0_floating[8];
extern volatile uint8_t motor1_commutation[8];
extern volatile uint8_t motor1_floating[8];
extern volatile int8_t index_0;
extern volatile int8_t index_1;
extern volatile uint8_t index_boot;
extern volatile int16_t counter_0;
extern volatile int16_t counter_2;
extern volatile bool timer1_enabled;

extern volatile int16_t motor0_speed;
extern volatile int16_t motor1_speed;


extern volatile bool boot;



// function pointers
typedef void (*func_ptr_t)();

extern volatile func_ptr_t ISR_timer0_compA_pointer;
extern volatile func_ptr_t ISR_timer0_compB_pointer;
extern volatile func_ptr_t ISR_timer1_compA_pointer;
extern volatile func_ptr_t ISR_timer2_compA_pointer;
extern volatile func_ptr_t ISR_uart_RX_pointer;


void Nullptr(void);

// Timer 0
void ISR_timer0_compA_boot_0(void);
void ISR_timer0_compA_boot_1(void);

void ISR_timer0_compA_main_forward(void);
void ISR_timer0_compA_main_reverse(void);
void ISR_timer0_compA_main_stop(void);

// Timer 1
void ISR_timer1_compA_boot(void);
void ISR_timer1_compA_main(void);

void Timer1_disable(void);
void Timer1_enable(void);

// Timer 2
void ISR_timer2_compA_boot_0(void);
void ISR_timer2_compA_boot_1(void);

void ISR_timer2_compA_main_forward(void);
void ISR_timer2_compA_main_reverse(void);
void ISR_timer2_compA_main_stop(void);

// UART
void ISR_UART_RX_0(void);
void ISR_UART_RX_1(void);
void ISR_UART_RX_2(void);
void ISR_UART_RX_3(void);

#endif