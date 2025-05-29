#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define F_CPU 16000000

#define LED_PIN PB5  // Arduino Uno: pin 13 = PORTB5

uint8_t ticks = 0;

void setupTimer0() {
  TCCR0A  = (1 << WGM01);               // CTC mode
  TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler = 1024
  OCR0A = 255;                         // Compare match value
  TIMSK0 = (1 << OCIE0A);              // Enable Timer1 compare match interrupt A
}

ISR(TIMER0_COMPA_vect) {
  ticks++;
  if (ticks >= 1) {                     // ~1 second
    PORTB ^= (1 << LED_PIN);             // Toggle LED
    ticks = 0;
  }
}

int main(void) {
  cli();

  // Set LED pin as output
  DDRB |= (1 << LED_PIN);

  // Disable unused peripherals
  power_adc_disable();
  power_spi_disable();
  power_timer1_disable();
  power_twi_disable();

  // Enable Timer0 only
  power_timer0_enable();

  setupTimer0();

  sei();

  while (1) {
    //idel
    sleep_mode();
  }
}
