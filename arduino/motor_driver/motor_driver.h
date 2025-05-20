#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define UBRR_VALUE ((F_CPU / (16UL * 9600)) - 1)
#define LED_PIN PB5
#define WINDING_0 PB5
#define WINDING_1 PB4
#define WINDING_2 PB3
#define ALL_Windings (1 << WINDING_0) | (1 << WINDING_1) | (1 << WINDING_2)
#define ROTATION_0 (1 << WINDING_0) | (1 << WINDING_1)
#define ROTATION_1 (1 << WINDING_1) | (1 << WINDING_2)
#define ROTATION_2 (1 << WINDING_0) | (1 << WINDING_2)



void setupTimer0() {
  // TCCR0A  = (1 << WGM00) | (1 << WGM01);               // CTC mode
  TCCR0B = (1 << CS02) | (1 << CS00); // | (1 << WGM02);  // Prescaler = 1024
  OCR0A = 254;                         // Compare match value
  TIMSK0 = (1 << TOIE0) | (1 << OCIE0A);
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

