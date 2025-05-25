

#define F_CPU 16000000
#define ARDUINO 10805

#include "motor_driver.h"

// Global variables
volatile uint8_t prescaler = 254;
volatile uint8_t motor0_commutation[7] = {MOTOR0_ROTATION0, MOTOR0_ROTATION1, MOTOR0_ROTATION2, MOTOR0_ROTATION3, MOTOR0_ROTATION4, MOTOR0_ROTATION5, 0};
volatile uint8_t motor1_commutation[7] = {MOTOR1_ROTATION0, MOTOR1_ROTATION1, MOTOR1_ROTATION2, MOTOR1_ROTATION3, MOTOR1_ROTATION4, MOTOR1_ROTATION5, 0};
volatile uint8_t index = 0;
volatile bool timer1_enabled = 0;

// Timer 0
// Overflow interupt
ISR(TIMER0_OVF_vect) {
  MOTOR0_PORT = 0;
  MOTOR1_PORT = 0;
}

// Compare A interupt
ISR(TIMER0_COMPA_vect) {
  MOTOR0_PORT = motor0_commutation[index];
  MOTOR1_PORT = motor1_commutation[index];
}

// Timer 1
// Compare A interupt
ISR(TIMER1_COMPA_vect) {
  prescaler = 20;
  index++;
  if (index > 6) {
    index = 0;
  }
}

// USART
// RX interupt resieving data
ISR(USART_RX_vect) {
  UCSR0B |= (1 << UDRIE0);
  prescaler = UDR0;

  if (prescaler > 250) {
    Timer1_disable();
    timer1_enabled = 0;
    MOTOR0_DDR &= ~(MOTOR0_ALL_SIGNALS);
    MOTOR1_DDR &= ~(MOTOR1_ALL_SIGNALS);
    index = 6;
  } 
  else {

    if (prescaler < 10) {
      prescaler = 10;
    }

    if (~timer1_enabled) {
      Timer1_enable();
      timer1_enabled = 1;
      MOTOR0_DDR |= MOTOR0_ALL_SIGNALS;
      MOTOR1_DDR |= MOTOR1_ALL_SIGNALS;
    }
    
    uint16_t temp = prescaler*100;
    OCR1A = temp;
    if (TCNT1 > temp) {
      TCNT1 = temp - 10;
    }
  }
}

// TX interupt
// runs every time TX buffer is readdy
ISR(USART_UDRE_vect) {
  UCSR0B &= ~(1 << UDRIE0);
  UDR0 = prescaler;
}


int main(void) {
  cli();

  // Disable unused peripherals
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  
  // Enable Timer0 only
  power_timer0_enable();
  power_timer1_enable();
  power_usart0_enable();

  // setting upp peripherals
  
  setupTimer0();

  setupTimer1();
  timer1_enabled = 1;
  Timer1_disable();
  timer1_enabled = 0;
  MOTOR0_DDR &= ~(MOTOR0_ALL_SIGNALS);
  MOTOR1_DDR &= ~(MOTOR1_ALL_SIGNALS);
  index = 6;

  USART_init();

  sei();

  while (1) {
    //idel
    sleep_mode();
  }
}