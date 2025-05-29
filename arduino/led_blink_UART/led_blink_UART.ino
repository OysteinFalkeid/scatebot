#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define F_CPU 16000000
#define ARDUINO 10805

#define UBRR_VALUE ((F_CPU / (16UL * 9600)) - 1)
#define LED_PIN PB5
#define WINDING_0 PB5
#define WINDING_1 PB4
#define WINDING_2 PB3
#define ALL_Windings (1 << WINDING_0) | (1 << WINDING_1) | (1 << WINDING_2)
#define ROTATION_0 (1 << WINDING_0) | (1 << WINDING_1)
#define ROTATION_1 (1 << WINDING_1) | (1 << WINDING_2)
#define ROTATION_2 (1 << WINDING_0) | (1 << WINDING_2)

uint8_t prescaler = 56;

uint8_t my_values[3] = {ROTATION_0, ROTATION_1, ROTATION_2};

uint8_t ticks = 0;
uint8_t index = 0;

void setupTimer0() {
  TCCR0A  = (1 << WGM01);               // CTC mode
  TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler = 1024
  OCR0A = 255;                         // Compare match value
  TIMSK0 = (1 << OCIE0A);              // Enable Timer1 compare match interrupt A
}

ISR(TIMER0_COMPA_vect) {
  ticks++;
  if (ticks >= prescaler) {                     // ~1 second
    // PORTB ^= (1 << LED_PIN);             // Toggle LED
    PORTB ^= my_values[index];
    ticks = 0;
  }
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

ISR(USART_RX_vect) {
  UCSR0B |= (1 << UDRIE0);
  prescaler = UDR0;
}

ISR(USART_UDRE_vect) {
  UCSR0B &= ~(1 << UDRIE0);
  UDR0 = prescaler;
}

int main(void) {
  cli();

  // Set LED pin as output
  // DDRB |= (1 << LED_PIN);
  DDRB |= ALL_Windings;

  // Disable unused peripherals
  power_adc_disable();
  power_spi_disable();
  power_timer1_disable();
  power_twi_disable();

  // Enable Timer0 only
  power_timer0_enable();

  setupTimer0();
  USART_init();

  sei();

  while (1) {
    //idel
    sleep_mode();
    // uint8_t byte = USART_receive();  // Block until byte is received
    // prescaler = byte;
    // USART_transmit(byte);           // Echo back
  }
}