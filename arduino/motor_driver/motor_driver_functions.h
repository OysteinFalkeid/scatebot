#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "defines.h"
#include "motor_driver_setup.h"

// Global variables
volatile uint8_t prescaler = 254;
volatile uint8_t motor0_commutation[7] = {MOTOR0_ROTATION5, MOTOR0_ROTATION4, MOTOR0_ROTATION3, MOTOR0_ROTATION2, MOTOR0_ROTATION1, MOTOR0_ROTATION0, 0};
volatile uint8_t motor1_commutation[7] = {MOTOR1_ROTATION0, MOTOR1_ROTATION1, MOTOR1_ROTATION2, MOTOR1_ROTATION3, MOTOR1_ROTATION4, MOTOR1_ROTATION5, 0};
volatile uint8_t index = 0;
volatile bool timer1_enabled = 0;


volatile bool boot;


// function pointers
void (*ISR_timer0_compA_pointer)() = nullptr;
void (*ISR_timer1_compA_pointer)() = nullptr;
void (*ISR_timer2_compA_pointer)() = nullptr;


void ISR_timer0_compA_boot_0(void) {
    MOTOR0_PORT = motor0_commutation[index];
    index++;
    ISR_timer0_compA_pointer = ISR_timer0_compA_boot_1;
}

void ISR_timer0_compA_boot_1(void) {
    MOTOR0_PORT = motor0_commutation[index];
    index--;
    ISR_timer0_compA_pointer = ISR_timer0_compA_boot_0;
}

void ISR_timer0_compA_main(void) {
    MOTOR0_PORT = motor0_commutation[index];
}

void ISR_timer1_compA_boot(void) {
    boot = false;
    ISR_timer1_compA_pointer = ISR_timer1_compA_main;
}

void ISR_timer1_compA_main(void) {
    index++;
    if (index > 5) {
        index = 0;
  }
}

void Timer1_disable(void) {
    TCCR1B = 0;
}

void Timer1_enable(void) {
    TCCR1B = TCCR1_PRESCALER_1024_MASK | (1 << WGM12);
}

void ISR_timer2_compA_boot_0(void) {
    MOTOR1_PORT = motor1_commutation[index];
    index++;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_1;
}

void ISR_timer2_compA_boot_1(void) {
    MOTOR1_PORT = motor1_commutation[index];
    index--;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_0;
}

void ISR_timer2_compA_main(void) {
    MOTOR1_PORT = motor1_commutation[index];
}