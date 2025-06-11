#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "defines.h"
#include "motor_driver_setup.h"
#include "motor_driver_functions.h"


// Global variables
volatile uint8_t prescaler = 254;
volatile uint8_t motor0_commutation[8] = {MOTOR0_ROTATION5, MOTOR0_ROTATION4, MOTOR0_ROTATION3, MOTOR0_ROTATION2, MOTOR0_ROTATION1, MOTOR0_ROTATION0, 0, 0};
volatile uint8_t motor0_floating[8] = {MOTOR0_FLOATING5, MOTOR0_FLOATING4, MOTOR0_FLOATING3, MOTOR0_FLOATING2, MOTOR0_FLOATING1, MOTOR0_FLOATING0, 0, 0};
volatile uint8_t motor1_commutation[8] = {MOTOR1_ROTATION0, MOTOR1_ROTATION1, MOTOR1_ROTATION2, MOTOR1_ROTATION3, MOTOR1_ROTATION4, MOTOR1_ROTATION5, 0, 0};
volatile uint8_t motor1_floating[8] = {MOTOR1_FLOATING0, MOTOR1_FLOATING1, MOTOR1_FLOATING2, MOTOR1_FLOATING3, MOTOR1_FLOATING4, MOTOR1_FLOATING5, 0, 0};

volatile int8_t index_0 = 0;
volatile int8_t index_1 = 0;
volatile int16_t counter_0 = 0;
volatile int16_t counter_2 = 0;
volatile bool timer1_enabled = 0;

volatile int16_t motor0_speed = 0;
volatile int16_t motor1_speed = 0;


// function pointers
typedef void (*func_ptr_t)();
volatile func_ptr_t ISR_timer0_compA_pointer = nullptr;
volatile func_ptr_t ISR_timer0_compB_pointer = nullptr;
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
    MOTOR0_PORT = motor0_commutation[index_0];
    sei();
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
    MOTOR0_PORT = motor0_commutation[index_0];
    sei();
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
    index_0 = 6;
    MOTOR0_PORT = motor0_commutation[index_0];
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

void ISR_timer1_compA_UCSR0B(void) {
    UCSR0B |= (1 << UDRIE0);
}

// Timer 2
void ISR_timer2_compA_boot_0(void) {
    MOTOR1_PORT = motor1_commutation[index_1];
    index_1++;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_1;
}

void ISR_timer2_compA_boot_1(void) {
    MOTOR1_PORT = motor1_commutation[index_1];
    index_1--;
    ISR_timer2_compA_pointer = ISR_timer2_compA_boot_0;
}

void ISR_timer2_compA_main_forward(void) {
    MOTOR1_PORT = motor1_commutation[index_1];
    sei();
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
    sei();
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
    index_1 = 6;
    MOTOR1_PORT = motor1_commutation[index_0];
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
    int8_t data;
    data = UDR0;
    motor0_speed = ((data & (1 << 7)) << 8) | (data << 7);
    // // UCSR0B |= (1 << UDRIE0);
    // UDR0 = (motor0_speed >> 7);
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
    int8_t data;
    data = UDR0;
    motor1_speed = ((data & (1 << 7)) << 8) | (data << 7);
    // // UCSR0B |= (1 << UDRIE0);
    // UDR0 = (motor1_speed >> 7);
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