

#define F_CPU 16000000
#define ARDUINO 10805

#include "motor_driver.h"


volatile uint8_t prescaler = 254;
volatile uint8_t my_values[4] = {(MOTOR0_ROTATION0 | MOTOR1_ROTATION0), (MOTOR0_ROTATION1 | MOTOR1_ROTATION1), (MOTOR0_ROTATION2 | MOTOR1_ROTATION2), 0};
volatile uint8_t index = 0;
volatile bool timer1_enabled = 0;

ISR(TIMER0_OVF_vect) {
  MOTOR_PORT = my_values[index];
}

ISR(TIMER0_COMPA_vect) {
  MOTOR_PORT = 0;
}

ISR(TIMER1_COMPA_vect) {
  prescaler = 20;
  index++;
  if (index > 2) {
    index = 0;
  }
}

ISR(USART_RX_vect) {
  UCSR0B |= (1 << UDRIE0);
  prescaler = UDR0;

  if (prescaler > 250) {
    Timer1_disable();
    timer1_enabled = 0;
    MOTOR_DDR &= ~(MOTOR0_ALL_Windings | MOTOR1_ALL_Windings);
    index = 3;
  } 
  else {

    if (prescaler < 10) {
      prescaler = 10;
    }

    if (~timer1_enabled) {
      Timer1_enable();
      timer1_enabled = 1;
      MOTOR_DDR |= MOTOR1_ALL_Windings;
      MOTOR_DDR |= MOTOR0_ALL_Windings;
    }
    
    uint16_t temp = prescaler*100;
    OCR1A = temp;
    if (TCNT1 > temp) {
      TCNT1 = temp - 10;
    }
  }
}

ISR(USART_UDRE_vect) {
  UCSR0B &= ~(1 << UDRIE0);
  UDR0 = prescaler;
}


int main(void) {
  cli();

  // Set LED pin as output
  // DDRB |= (1 << LED_PIN);
  // MOTOR_DDR |= MOTOR1_ALL_Windings;
  // MOTOR_DDR |= MOTOR0_ALL_Windings;

  // Disable unused peripherals
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  
  // Enable Timer0 only
  power_timer0_enable();
  power_timer1_enable();
  power_usart0_enable();

  setupTimer0();
  setupTimer1();
  timer1_enabled = 1;
  Timer1_disable();
  timer1_enabled = 0;
  MOTOR_DDR &= ~(MOTOR0_ALL_Windings | MOTOR1_ALL_Windings);
  index = 3;
  USART_init();

  sei();

  while (1) {
    //idel
    sleep_mode();
  }
}