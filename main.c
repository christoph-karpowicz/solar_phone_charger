#define F_CPU 37500

#include <avr/interrupt.h>
#include <stdbool.h>
#include "display.h"

#define V_CHARGING_THRESHOLD 4
#define ARRAY_COUNTER_MAX 9
#define WAIT_COUNTER_MAX 10

#define all_above_threshold(reads) (reads == 1023)
#define all_below_threshold(reads) (reads == 0)

volatile int8_t last_digit = -1;
volatile int8_t last_with_dot = -1;
volatile uint8_t array_counter = 0;
volatile uint8_t wait_counter = 0;
volatile uint8_t no_change_counter = 0;
volatile uint16_t digit_reads = 0;

volatile bool adc_processing = false;
volatile bool waiting = false;
volatile bool display_loader = false;
volatile uint16_t adc_result = 0;

void init() {
    // Timer/Counter Output Compare Match B Interrupt Enable
    TIMSK0 |= _BV(OCIE0B);
    // set CTC timer mode
    TCCR0A |= _BV(WGM01);
    // set the 1024 timer prescaler
    TCCR0B |= _BV(CS02) | _BV(CS00);

    // enable ADC with 64 prescaler division factor and auto trigger, ADC interrupt enabled
    ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN);
    // select PB4 (ADC2) analog channel with AVCC ref
    ADMUX = _BV(MUX1);

    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);
    // init driver
    DDRB |= _BV(PB3);

    // set the system clock prescaler to 256
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS3);

    // Enable global interrupts
    sei();
}

uint8_t is_charging_on() {
    return PINB & _BV(PB3);
}

void charging_on() {
    PORTB |= _BV(PB3);
}

void charging_off() {
    PORTB &= ~_BV(PB3);
}

uint16_t adc_read() {
    // start conversion
    ADCSRA |= _BV(ADSC);
    // wait for end of conversion
    while (ADCSRA & _BV(ADSC));
    return ADC;
}

void process_adc_read(const uint16_t adc_result_val) {
    adc_processing = true;
    uint8_t digit;
    uint8_t with_dot;
    if (adc_result_val >= 1000){
        digit = 5;
        with_dot = 0;
    } else if (adc_result_val >= 921) {
        digit = 4;
        with_dot = 1;
    } else if (adc_result_val >= 820) {
        digit = 4;
        with_dot = 0;
    } else if (adc_result_val >= 716) {
        digit = 3;
        with_dot = 1;
    } else if (adc_result_val >= 615) {
        digit = 3;
        with_dot = 0;
    } else if (adc_result_val >= 512) {
        digit = 2;
        with_dot = 1;
    } else if (adc_result_val >= 410) {
        digit = 2;
        with_dot = 0;
    } else if (adc_result_val >= 307) {
        digit = 1;
        with_dot = 1;
    } else if (adc_result_val >= 205) {
        digit = 1;
        with_dot = 0;
    } else if (adc_result_val >= 102) {
        digit = 0;
        with_dot = 1;
    } else {
        digit = 0;
        with_dot = 0;
    }

    if (digit >= V_CHARGING_THRESHOLD) {
        digit_reads |= (1 << array_counter);
    } else {
        digit_reads &= ~(1 << array_counter);
    }

    if (display_loader) {
        display(1 << (array_counter - 1));
        last_digit = -1;
        last_with_dot = -1;
    } else if (digit != last_digit || with_dot != last_with_dot) {
        display_number(digit, with_dot);
        last_digit = digit;
        last_with_dot = with_dot;
        no_change_counter = 0;
    } else if (no_change_counter == 200) {
        display_empty();
    } else {
        no_change_counter++;
    }

    if (array_counter == ARRAY_COUNTER_MAX) {
        const bool charging_is_on = is_charging_on();
        const bool can_start_charging = !charging_is_on && all_above_threshold(digit_reads);
        if (!waiting && can_start_charging) {
            display_loader = false;
            charging_on();
        } else if (waiting && can_start_charging) {
            display_loader = true;
        } else if (waiting && !can_start_charging) {
            display_loader = false;
        } else if (!waiting && charging_is_on && all_below_threshold(digit_reads)) {
            charging_off();
            waiting = true;
            wait_counter = 0;
        }
        if (waiting) {
            if (wait_counter == WAIT_COUNTER_MAX) {
                waiting = false;
                display_loader = false;
            } else {
                wait_counter++;
            }
        }
        array_counter = 0;
    } else {
        array_counter++;
    }

    adc_processing = false;
}


ISR(TIM0_COMPB_vect) {
    adc_result = adc_read();
}

int main(void) {
    init();
    while (1) {
        if (!adc_processing) {
            cli();
            process_adc_read(adc_result);
            sei();
        }
    }
    return 0;
}
