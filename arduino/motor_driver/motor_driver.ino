#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "motor_driver.h"

#include "defines.h"
#include "motor_driver_functions.h"
#include "motor_driver_setup.h"


// Timer 0
// Overflow interupt
ISR(TIMER0_OVF_vect) {
  MOTOR0_PORT = 0;
}

// Compare A interupt
ISR(TIMER0_COMPA_vect) {
  // MOTOR0_PORT = motor0_commutation[index_0];
  ISR_timer0_compA_pointer();
}

// Timer 1
// Compare A interupt
ISR(TIMER1_COMPA_vect) {
  ISR_timer1_compA_pointer();
}

// Timer 2
// Overflow interupt
ISR(TIMER2_OVF_vect) {
  MOTOR1_PORT = 0;
}

// Compare A interupt
ISR(TIMER2_COMPA_vect) {
  // MOTOR1_PORT = motor1_commutation[0];
  ISR_timer2_compA_pointer();
  // ISR_timer2_compA_main_forward();
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
  UDR0 = motor0_speed;
}


int main(void) {
  cli();

  setupPower();

  setupPorts_motor0();
  setupPorts_motor1();
  // MOTOR1_PORT |= (1 << 5);

  // ISR_timer0_compA_pointer = ISR_timer0_compA_boot_0;
  setupTimer0_8pre_interupts();
  // MOTOR1_PORT |= (1 << 5);

  //temporary timer used to control the index of the motors. this will be done in the PWM timer with some math on the cpu.
  // ISR_timer1_compA_pointer = ISR_timer1_compA_boot;
  // setupTimer1();
  // timer1_enabled = 1;
  // MOTOR1_PORT |= (1 << 5);
  
  // ISR_timer2_compA_pointer = ISR_timer2_compA_boot_0;
  SetupTimer2_8pre_interupts();
  // MOTOR1_PORT |= (1 << 5);
  
  // sei();
  
  // while (1) {
  //   // motors shold whine with a 2kHz freqancy
  //   sleep_mode();
  //   // MOTOR1_PORT |= (1 << 5);
  // }

  // cli();

  // Timer1_disable();
  // timer1_enabled = 0;
  // MOTOR0_DDR &= ~(MOTOR0_ALL_SIGNALS);
  // MOTOR1_DDR &= ~(MOTOR1_ALL_SIGNALS);
  // index_0 = 6;
  // index_1 = 6;
  // // MOTOR1_PORT |= (1 << 5);

  ISR_timer0_compA_pointer = ISR_timer0_compA_main_stop;
  // ISR_timer1_compA_pointer = Nullptr;
  ISR_timer2_compA_pointer = ISR_timer2_compA_main_stop;
  ISR_uart_RX_pointer = ISR_UART_RX_0;

  // MOTOR1_PORT |= (1 << 5);

  USART_init();
  // MOTOR1_PORT |= (1 << 5);

  
  sei();
  
  while (true) {
    //idel
    sleep_mode();
  }
}