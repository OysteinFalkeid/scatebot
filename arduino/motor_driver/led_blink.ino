#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define LED_PIN PB5  // Arduino Uno: pin 13 = PORTB5

void setupTimer2() {
  // Prescaler: 1024
  // Compare Match at OCR2A = 156 => ~1Hz blink (with 16 MHz clock)
  TCCR2A = (1 << WGM21);                // CTC mode
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler = 1024
  OCR2A = 156;                          // Compare match value
  TIMSK2 = (1 << OCIE2A);              // Enable Timer2 compare match interrupt
}

ISR(TIMER2_COMPA_vect) {
  // Toggle LED on interrupt
  PORTB ^= (1 << LED_PIN);
}

void setup() {
  cli(); // Disable global interrupts

  // Set LED pin as output
  DDRB |= (1 << LED_PIN);

  // Disable unused peripherals
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_twi_disable();

  // Enable Timer2 only
  power_timer2_enable();

  setupTimer2();

  sei(); // Enable global interrupts
}

void loop() {
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // Timer2 works in Power-save mode
  sleep_enable();
  sleep_cpu();  // Sleep until Timer2 interrupt fires
  sleep_disable();  // Resume execution here after ISR
}