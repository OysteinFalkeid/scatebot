#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define F_CPU 16000000
#define ARDUINO 10805

#include "defines.h"
#include "motor_driver_functions.h"


// Timer 0
// Overflow interupt
ISR(TIMER0_OVF_vect) {
  MOTOR0_PORT = 0;
}

// Compare A interupt
ISR(TIMER0_COMPA_vect) {
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
  ISR_timer2_compA_pointer();
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

  setupPower();
  
  setupTimer0_8pre_interupts();
  ISR_timer0_compA_pointer = ISR_timer0_compA_boot_0;

  //temporary timer used to control the index of the motors. this will be done in the PWM timer with some math on the cpu.
  setupTimer1();
  timer1_enabled = 1;
  ISR_timer1_compA_pointer = ISR_timer1_compA_boot;
  
  SetupTimer2_8pre_interupts();
  ISR_timer2_compA_pointer = ISR_timer2_compA_boot_0;
  
  sei();
  
  while (boot) {
    // motors shold whine with a 2kHz freqancy
  }

  cli();

  Timer1_disable();
  timer1_enabled = 0;
  MOTOR0_DDR &= ~(MOTOR0_ALL_SIGNALS);
  MOTOR1_DDR &= ~(MOTOR1_ALL_SIGNALS);
  index = 6;

  ISR_timer0_compA_pointer = ISR_timer0_compA_main;
  ISR_timer1_compA_pointer = ISR_timer1_compA_main;
  ISR_timer2_compA_pointer = ISR_timer2_compA_main;
  
  USART_init();
  
  sei();
  
  while (true) {
    //idel
    sleep_mode();
  }
}