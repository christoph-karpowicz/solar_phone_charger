#define F_CPU 37500

#include <avr/interrupt.h>
#include <stdbool.h>
#include "display.h"

#define V_CHARGING_THRESHOLD 4
#define ARRAY_COUNTER_MAX 9
#define WAIT_COUNTER_MAX 10
#define ADC_PROCESSING_FLAG 15
#define WAITING_FLAG 14
#define DISPLAY_LOADER_FLAG 13
#define LAST_WITH_DOT_FLAG 12

#define all_above_threshold(reads) ((reads & 1023) == 1023)
#define all_below_threshold(reads) ((reads & 1023) == 0)
#define set_flag(flag) (digit_reads_and_flags |= (1 << flag))
#define unset_flag(flag) (digit_reads_and_flags &= ~(1 << flag))
#define get_flag(flag) ((digit_reads_and_flags & (1 << flag)))

volatile int8_t last_digit = -1;
volatile uint8_t array_counter = 0;
volatile uint8_t wait_counter = 0;
volatile uint16_t digit_reads_and_flags = 0;

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
    set_flag(ADC_PROCESSING_FLAG);
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
        digit_reads_and_flags |= (1 << array_counter);
    } else {
        digit_reads_and_flags &= ~(1 << array_counter);
    }

    if (get_flag(DISPLAY_LOADER_FLAG)) {
        display(1 << (array_counter - 1));
        last_digit = -1;
        unset_flag(LAST_WITH_DOT_FLAG);
    } else if (digit != last_digit || with_dot != get_flag(LAST_WITH_DOT_FLAG)) {
        display_number(digit, with_dot);
        last_digit = digit;
        set_flag(LAST_WITH_DOT_FLAG);
    }

    if (array_counter == ARRAY_COUNTER_MAX) {
        const bool charging_is_on = is_charging_on();
        const bool can_start_charging = !charging_is_on && all_above_threshold(digit_reads_and_flags);
        if (!(get_flag(WAITING_FLAG)) && can_start_charging) {
            unset_flag(DISPLAY_LOADER_FLAG);
            charging_on();
        } else if (get_flag(WAITING_FLAG) && can_start_charging) {
            set_flag(DISPLAY_LOADER_FLAG);
        } else if (get_flag(WAITING_FLAG) && !can_start_charging) {
            unset_flag(DISPLAY_LOADER_FLAG);
        } else if (!(get_flag(WAITING_FLAG)) && charging_is_on && all_below_threshold(digit_reads_and_flags)) {
            charging_off();
            set_flag(WAITING_FLAG);
            wait_counter = 0;
        }
        if (get_flag(WAITING_FLAG)) {
            if (wait_counter == WAIT_COUNTER_MAX) {
                unset_flag(WAITING_FLAG);
                unset_flag(DISPLAY_LOADER_FLAG);
            } else {
                wait_counter++;
            }
        }
        array_counter = 0;
    } else {
        array_counter++;
    }

    unset_flag(ADC_PROCESSING_FLAG);
}


ISR(TIM0_COMPB_vect) {
    adc_result = adc_read();
}

int main(void) {
    init();
    while (1) {
        if (!get_flag(ADC_PROCESSING_FLAG)) {
            cli();
            process_adc_read(adc_result);
            sei();
        }
    }
    return 0;
}
