#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "motor_driver.h"

#include "defines.h"
#include "motor_driver_functions.h"
#include "motor_driver_setup.h"

uint8_t hall_sense = 0;


// Timer 0
// Overflow interupt
ISR(TIMER0_OVF_vect) {
  MOTOR0_PORT = motor0_floating[index_0];
  MOTOR1_PORT = motor1_floating[index_1];
}

// Compare A interupt
ISR(TIMER0_COMPA_vect) {
  // MOTOR0_PORT = motor0_commutation[index_0];
  ISR_timer0_compA_pointer();
}

// Compare B interupt
ISR(TIMER0_COMPB_vect) {
  // MOTOR0_PORT = motor0_commutation[index_0];
  ISR_timer0_compB_pointer();
}

// Timer 1
// Compare A interupt
ISR(TIMER1_COMPA_vect) {
  ISR_timer1_compA_pointer();
}

// USART
// RX interupt resieving data
ISR(USART_RX_vect) {
  ISR_uart_RX_pointer();
}

// TX interupt
// runs every time TX buffer is readdy
ISR(USART_UDRE_vect) {
  UCSR0B &= ~(1 << UDRIE0);
  UDR0 = hall_sense;
}


int main(void) {
  cli();

  setupPower();

  setupPorts_motor0();
  setupPorts_motor1();

  setupTimer0_8pre_interupts();
  setupTimer1_UCSR0B();
  USART_init();

  ISR_timer0_compA_pointer = ISR_timer0_compA_main_stop;
  ISR_timer0_compB_pointer = ISR_timer2_compA_main_stop;
  ISR_uart_RX_pointer = ISR_UART_RX_0;
  ISR_timer1_compA_pointer = ISR_timer1_compA_UCSR0B;


  DDRC = 0; // set all of port D as innput 
  PORTC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);

  sei();
  
  while (true) {
    //idel
    sleep_mode();
    hall_sense = PINC;
  }
}