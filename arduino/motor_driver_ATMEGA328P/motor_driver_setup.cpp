#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "defines.h"
#include "motor_driver_setup.h"

#define UBRR_VALUE ((F_CPU / (16UL * 9600)) - 1)

void setupPower(void) {
    // Disable unused peripherals
    power_adc_disable();
    power_spi_disable();
    power_twi_disable();
    // power_timer2_disable();
    
    // enable power
    power_timer0_enable();
    power_timer1_enable();
    power_timer2_enable();
    power_usart0_enable();
}

void setupPorts_motor0(void) {
    MOTOR0_DDR |= MOTOR0_ALL_SIGNALS;
}

void setupPorts_motor1(void) {
    MOTOR1_DDR |= MOTOR1_ALL_SIGNALS;
}

void setupTimer0_8pre_interupts(void) {
    // 8 prescaler
    TCCR0B = TCCR0_PRESCALER_8_MASK;
    // compare value of A
    OCR0A = 253;    
    // // compare value of B
    // OCR0B = 245;
    TCNT0 = 0;
    // enable interupts
    TIMSK0 = (1 << TOIE0) | (1 << OCIE0A);// | (1 << OCIE0B);
}

void setupTimer0_0pre_interupts(void) {
    // 8 prescaler
    TCCR0B = TCCR0_PRESCALER_0_MASK;
    // compare value of A
    OCR0A = 253;   
    // enable interupts
    TIMSK0 = (1 << TOIE0) | (1 << OCIE0A) | (1 << OCIE0B);
}

void setupTimer0_0pre_pwm(void) {
    TCCR0B = TCCR0_PRESCALER_0_MASK;
    TCCR0A = (1 << WGM00) | (1 << WGM01);
    OCR0A = UINT8_MAX / 2;
    OCR0B = UINT8_MAX / 2;
    TIMSK0 = (1 << TOIE0);
}

void setupTimer1(void) {
    TCCR1A = 0;
    TCCR1B = TCCR1_PRESCALER_1024_MASK | (1 << WGM12);
    TIMSK1 = (1 << OCIE1A);
    OCR1A = UINT16_MAX-1;
    TCNT1 = 0;
}

void setupTimer1_UCSR0B(void) {
    TCCR1A = 0;
    TCCR1B = TCCR1_PRESCALER_1024_MASK; // prescaler
    TCCR1B |= (1 << WGM12); // CTC
    TIMSK1 = (1 << OCIE1A); // COMP A interupt enable
    OCR1A = 157; // compare value
    TCNT1 = 0; // actual counted value restarting the counter
}


void SetupTimer2_8pre_interupts(void) {
    // 8 prescaler
    TCCR2B = TCCR2_PRESCALER_8_MASK;
    // compare value of A
    OCR2A = 254; 
    TCNT2 = 0;
    // enable interupts
    TIMSK2 = (1 << TOIE2) | (1 << OCIE2A);// | (1 << OCIE2B);
}


void USART_init(void) {
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE);

    // Enable receiver and transmitter and interupt on RX and empty TX buffer
    UCSR0B = (1 << RXEN0);
    UCSR0B |= (1 << TXEN0);
    UCSR0B |= (1 << UDRIE0);
    UCSR0B |= (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

