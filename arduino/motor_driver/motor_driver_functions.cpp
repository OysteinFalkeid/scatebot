#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "defines.h"
#include "motor_driver_setup.h"
#include "motor_driver_functions.h"

// is sinusodial waveform is wanted this is an array form 0 to pi/2 with 255 steps.
// created by gpt
// TODO manualy calculate to ensure corect table values
#include <avr/pgmspace.h>
const uint8_t sine_table[128] PROGMEM = {
  0, 6, 13, 19, 25, 31, 38, 44, 50, 56, 62, 68, 74, 80, 86, 92,
  98,104,109,115,120,126,131,137,142,147,152,157,162,167,172,176,
  181,185,190,194,198,202,206,210,213,217,220,224,227,230,233,236,
 239,241,244,246,248,250,252,254,255,255,256,256,256,256,255,254,
 253,251,249,247,244,242,239,236,232,229,225,221,217,213,208,204,
 199,194,189,183,178,172,166,160,154,148,142,135,129,122,116,109,
 102, 95, 88, 81, 74, 67, 60, 53, 46, 39, 32, 25, 19, 12, 6, 0
};


// Global variables
volatile uint8_t prescaler = 254;
volatile uint8_t motor0_commutation[7] = {MOTOR0_ROTATION5, MOTOR0_ROTATION4, MOTOR0_ROTATION3, MOTOR0_ROTATION2, MOTOR0_ROTATION1, MOTOR0_ROTATION0, 0};
volatile uint8_t motor1_commutation[7] = {MOTOR1_ROTATION0, MOTOR1_ROTATION1, MOTOR1_ROTATION2, MOTOR1_ROTATION3, MOTOR1_ROTATION4, MOTOR1_ROTATION5, 0};
volatile int8_t index_0 = 0;
volatile int8_t index_1 = 0;
volatile uint8_t index_boot = 0;
volatile int16_t counter_0 = 0;
volatile int16_t counter_2 = 0;
volatile bool timer1_enabled = 0;

volatile int16_t motor0_speed = (INT16_MAX -1)/8;
volatile int16_t motor1_speed = (INT16_MAX -1)/8;


volatile bool boot;


// function pointers
typedef void (*func_ptr_t)();
volatile func_ptr_t ISR_timer0_compA_pointer = nullptr;
volatile func_ptr_t ISR_timer1_compA_pointer = nullptr;
volatile func_ptr_t ISR_timer2_compA_pointer = nullptr;
volatile func_ptr_t ISR_uart_RX_pointer = nullptr;


void Nullptr(void) {
    // Do nothing
}

// Timer 0
void ISR_timer0_compA_boot_0(void) {
    MOTOR0_PORT |= motor0_commutation[index_0];
    index_0++;
    ISR_timer0_compA_pointer = ISR_timer0_compA_boot_1;
}

void ISR_timer0_compA_boot_1(void) {
    MOTOR0_PORT |= motor0_commutation[index_0];
    index_0--;
    ISR_timer0_compA_pointer = ISR_timer0_compA_boot_0;
}

void ISR_timer0_compA_main_forward(void) {
    MOTOR0_PORT |= motor0_commutation[index_0];
    counter_0++;
    if (counter_0 > motor0_speed) {
        counter_0 = 0;
        index_0++;
        if (index_0 > 5) {
            index_0 = 0;
        }    
    }
}

void ISR_timer0_compA_main_reverse(void) {
    MOTOR0_PORT |= motor0_commutation[index_0];
    counter_0--;
    if (counter_0 < motor0_speed) {
        counter_0 = 0;
        index_0--;
        if (index_0 < 0) {
            index_0 = 5;
        }    
    }
}

void ISR_timer0_compA_main_stop(void) {
    MOTOR0_PORT = 0;
}

// Timer 1
void ISR_timer1_compA_boot(void) {
    index_boot++;
    if (index_boot > 0) {
        boot = false;
        ISR_timer1_compA_pointer = Nullptr;
    }
}

void ISR_timer1_compA_main(void) {
    index_0++;
    if (index_0 > 5) {
        index_0 = 0;
    }

    index_1++;
    if (index_1 > 5) {
        index_1 = 0;
    }
}

void Timer1_disable(void) {
    TCCR1B = 0;
}

void Timer1_enable(void) {
    TCCR1B = TCCR1_PRESCALER_1024_MASK | (1 << WGM12);
}

// Timer 2
void ISR_timer2_compA_boot_0(void) {
    MOTOR1_PORT |= motor1_commutation[index_1];
    index_1++;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_1;
}

void ISR_timer2_compA_boot_1(void) {
    MOTOR1_PORT |= motor1_commutation[index_1];
    index_1--;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_0;
}

void ISR_timer2_compA_main_forward(void) {
    MOTOR1_PORT |= motor1_commutation[index_1];
    counter_2++;
    if (counter_2 > motor1_speed) {
        counter_2 = 0;
        index_1++;
        if (index_1 > 5) {
            index_1 = 0;
        }    
    }
}

void ISR_timer2_compA_main_reverse(void) {
    MOTOR1_PORT = motor1_commutation[index_1];
    counter_2--;
    if (counter_2 < motor1_speed) {
        counter_2 = 0;
        index_1--;
        if (index_1 < 0) {
            index_1 = 5;
        }    
    }
}

void ISR_timer2_compA_main_stop(void) {
    MOTOR1_PORT = 0;
}

// UART
void ISR_UART_RX_0(void) {
    int8_t data;
    data = UDR0;
    // UCSR0B |= (1 << UDRIE0);
    if (data = UINT8_MAX) {
        ISR_uart_RX_pointer = ISR_UART_RX_1;
    }
}

void ISR_UART_RX_1(void) {
    motor0_speed = (UDR0 << 7);
    // UCSR0B |= (1 << UDRIE0);
    if (motor0_speed > 0) {
        ISR_timer0_compA_pointer = ISR_timer0_compA_main_forward;
    }else if (motor0_speed < 0) {
        ISR_timer0_compA_pointer = ISR_timer0_compA_main_reverse;
    }else {
        ISR_timer0_compA_pointer = ISR_timer0_compA_main_stop;
    }
    ISR_uart_RX_pointer = ISR_UART_RX_2;
}

void ISR_UART_RX_2(void) {
    motor1_speed = (UDR0 << 7);
    // UCSR0B |= (1 << UDRIE0);
    if (motor1_speed > 0) {
        ISR_timer2_compA_pointer = ISR_timer2_compA_main_forward;
    }else if (motor0_speed < 0) {
        ISR_timer2_compA_pointer = ISR_timer2_compA_main_reverse;
    }else {
        ISR_timer2_compA_pointer = ISR_timer2_compA_main_stop;
    }
    ISR_uart_RX_pointer = ISR_UART_RX_3;
}

void ISR_UART_RX_3(void) {
    int8_t data;
    data = UDR0;
    // UCSR0B |= (1 << UDRIE0);
    if (data == 0) {
        ISR_uart_RX_pointer = ISR_UART_RX_0;
    }
}