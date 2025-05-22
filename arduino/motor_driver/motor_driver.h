#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define UBRR_VALUE ((F_CPU / (16UL * 9600)) - 1)
#define LED_PIN PB5
#define MOTOR1_WINDONG0 PB5
#define MOTOR1_WINDONG1 PB4
#define MOTOR1_WINDONG2 PB3
#define MOTOR1_ALL_Windings (1 << MOTOR1_WINDONG0) | (1 << MOTOR1_WINDONG1) | (1 << MOTOR1_WINDONG2)
#define MOTOR1_ROTATION0 (1 << MOTOR1_WINDONG0) | (1 << MOTOR1_WINDONG1)
#define MOTOR1_ROTATION1 (1 << MOTOR1_WINDONG1) | (1 << MOTOR1_WINDONG2)
#define MOTOR1_ROTATION2 (1 << MOTOR1_WINDONG0) | (1 << MOTOR1_WINDONG2)

#define MOTOR0_WINDONG0 PB2
#define MOTOR0_WINDONG1 PB1
#define MOTOR0_WINDONG2 PB0
#define MOTOR0_ALL_Windings (1 << MOTOR0_WINDONG0) | (1 << MOTOR0_WINDONG1) | (1 << MOTOR0_WINDONG2)
#define MOTOR0_ROTATION0 (1 << MOTOR0_WINDONG0) | (1 << MOTOR0_WINDONG1)
#define MOTOR0_ROTATION1 (1 << MOTOR0_WINDONG1) | (1 << MOTOR0_WINDONG2)
#define MOTOR0_ROTATION2 (1 << MOTOR0_WINDONG2) | (1 << MOTOR0_WINDONG2)

#define MOTOR_PORT PORTB
#define MOTOR_DDR DDRB


#define MOTOR1_WINDONG0 PB5
#define MOTOR1_WINDONG1 PB4
#define MOTOR1_WINDONG2 PB3
#define MOTOR1_ALL_Windings (1 << MOTOR1_WINDONG0) | (1 << MOTOR1_WINDONG1) | (1 << MOTOR1_WINDONG2)
#define MOTOR1_ROTATION0 (1 << MOTOR1_WINDONG0)
#define MOTOR1_ROTATION1 (1 << MOTOR1_WINDONG1)
#define MOTOR1_ROTATION2 (1 << MOTOR1_WINDONG2)

#define MOTOR0_WINDONG0 PB2
#define MOTOR0_WINDONG1 PB1
#define MOTOR0_WINDONG2 PB0
#define MOTOR0_ALL_Windings (1 << MOTOR0_WINDONG0) | (1 << MOTOR0_WINDONG1) | (1 << MOTOR0_WINDONG2)
#define MOTOR0_ROTATION0 (1 << MOTOR0_WINDONG0)
#define MOTOR0_ROTATION1 (1 << MOTOR0_WINDONG1)
#define MOTOR0_ROTATION2 (1 << MOTOR0_WINDONG2)

#define MOTOR_PORT PORTB
#define MOTOR_DDR DDRB

void setupTimer0(void) {
    // TCCR0A  = (1 << WGM00) | (1 << WGM01);               // CTC mode
    // TCCR0B = (1 << CS02) | (1 << CS00); // | (1 << WGM02);  // Prescaler = 1024
    // TCCR0B = (1 << CS00); // no prescaler
    TCCR0B = (1 << CS01); // 8 prescaler
    // TCCR0B = (1 << CS01) | (1 << CS00); // 64 prescaler
    // TCCR0B = (1 << CS02); // 256 prescaler
    // TCCR0B = (1 << CS02) | (1 << CS00); // 64 prescaler
    OCR0A = 200;                        // Compare match value
    TIMSK0 = (1 << TOIE0) | (1 << OCIE0A);
}

void setupTimer1(void) {
    TCCR1A = 0;
    TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);
    TIMSK1 = (1 << OCIE1A);
    OCR1A = 15624;
    TCNT1 = 0;
}

void Timer1_disable(void) {
    // TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    TCCR1B = 0;
}

void Timer1_enable(void) {
    // TCCR1B |= (1 << CS12) | (1 << CS10);
    TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);
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

