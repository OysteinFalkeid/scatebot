#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#ifndef MOTOR_SETUP
#define MOTOR_SETUP

#include "defines.h"

#define UBRR_VALUE ((F_CPU / (16UL * 9600)) - 1)

void setupPower(void);

void setupPorts_motor0(void);

void setupPorts_motor1(void);

void setupTimer0_8pre_interupts(void);
void setupTimer0_0pre_interupts(void);
void setupTimer0_0pre_pwm(void);

void setupTimer1(void);
void setupTimer1_UCSR0B(void);

void SetupTimer2_8pre_interupts(void);

void USART_init(void);

#endif