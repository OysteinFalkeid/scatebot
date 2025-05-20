

#define F_CPU 16000000
#define ARDUINO 10805

#include "motor_driver.h"


uint8_t prescaler = 56;
uint8_t my_values[3] = {ROTATION_0, ROTATION_1, ROTATION_2};
uint8_t index = 0;

ISR(TIMER0_OVF_vect) {
  PORTB = ALL_Windings;
}

ISR(TIMER0_COMPA_vect) {
  PORTB = 0;
}

ISR(USART_RX_vect) {
  UCSR0B |= (1 << UDRIE0);
  prescaler = UDR0;
  OCR0A = prescaler;
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