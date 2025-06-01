#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#ifndef MOTOR
#define MOTOR

#define F_CPU 16000000
#define ARDUINO 10805

#include "defines.h"
#include "motor_driver_functions.h"

// Timer 0
// Overflow interupt
ISR(TIMER0_OVF_vect);

// Compare A interupt
ISR(TIMER0_COMPA_vect);

// Timer 1
// Compare A interupt
ISR(TIMER1_COMPA_vect);

// Timer 2
// Overflow interupt
ISR(TIMER2_OVF_vect);

// Compare A interupt
ISR(TIMER2_COMPA_vect);

// USART
// RX interupt resieving data
ISR(USART_RX_vect);

// TX interupt
// runs every time TX buffer is readdy
ISR(USART_UDRE_vect);

#endif