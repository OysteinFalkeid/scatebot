#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#ifndef MOTOR_MASKS
#define MOTOR_MASKS

// TCCR0B Timer 0 clock controll
#define TCCR0_NO_CLOCK_MASK         0
#define TCCR0_PRESCALER_0_MASK      (1 << CS00)
#define TCCR0_PRESCALER_8_MASK       (1 << CS01)
#define TCCR0_PRESCALER_64_MASK      (1 << CS01) | (1 << CS00)
#define TCCR0_PRESCALER_256_MASK    (1 << CS02)
#define TCCR0_PRESCALER_1024_MASK   (1 << CS02) | (1 << CS00)
#define TCCR0_EXTERN_FALLING_MASK   (1 << CS02) | (1 << CS01)
#define TCCR0_EXTERN_RISING_MASK    (1 << CS02) | (1 << CS01) | (1 << CS00)


// TCCR1B Timer 1 clock controll
#define TCCR1_NO_CLOCK_MASK         0
#define TCCR1_PRESCALER_0_MASK      (1 << CS10)
#define TCCR1_PRESCALER_8_MASK       (1 << CS11)
#define TCCR1_PRESCALER_64_MASK      (1 << CS11) | (1 << CS10)
#define TCCR1_PRESCALER_256_MASK    (1 << CS12)
#define TCCR1_PRESCALER_1024_MASK   (1 << CS12) | (1 << CS10)
#define TCCR1_EXTERN_FALLING_MASK   (1 << CS12) | (1 << CS11)
#define TCCR1_EXTERN_RISING_MASK    (1 << CS12) | (1 << CS11) | (1 << CS10)


// TCCR2B Timer 2 clock controll
#define TCCR2_NO_CLOCK_MASK         0
#define TCCR2_PRESCALER_0_MASK      (1 << CS20)
#define TCCR2_PRESCALER_8_MASK       (1 << CS21)
#define TCCR2_PRESCALER_64_MASK      (1 << CS21) | (1 << CS20)
#define TCCR2_PRESCALER_256_MASK    (1 << CS22)
#define TCCR2_PRESCALER_1024_MASK   (1 << CS22) | (1 << CS20)
#define TCCR2_EXTERN_FALLING_MASK   (1 << CS22) | (1 << CS21)
#define TCCR2_EXTERN_RISING_MASK    (1 << CS22) | (1 << CS21) | (1 << CS20)

#endif