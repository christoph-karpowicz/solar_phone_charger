#define F_CPU 37500

#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "display.h"

#define V_CHARGING_THRESHOLD 4

volatile int8_t last_digit;
volatile int8_t last_with_dot;
volatile int8_t digit_reads[10];
volatile uint8_t array_counter = 0;
volatile uint8_t wait_counter = 0;

volatile bool adc_processing;
volatile bool wait;
volatile uint16_t adc_result;

void init() {
    // Timer/Counter Output Compare Match B Interrupt Enable
    TIMSK0 |= _BV(OCIE0B);
    // set the timer's TOP value
    OCR0B = 256;
    // set CTC timer mode
    TCCR0A |= _BV(WGM01);
    // set the 1024 timer prescaler
    TCCR0B |= _BV(CS02) | _BV(CS00);

    // enable ADC with 64 prescaler division factor and auto trigger, ADC interrupt enabled
    // ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE);
    ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN);
    // set ADC auto trigger source to Timer/Counter Compare Match B
    // ADCSRB |= _BV(ADTS2) | _BV(ADTS0);
    // select PB4 (ADC2) analog channel with AVCC ref
    ADMUX = _BV(MUX1);
    // disable ADC
    // ADCSRA &= ~_BV(ADEN);

    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);
    // init driver
    DDRB |= _BV(PB3);

    // set the system clock prescaler to 256
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS3);

    last_digit = -1;
    last_with_dot = -1;
    adc_result = 0;
    wait = false;
    adc_processing = false;
    display_number(3,1);

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

bool every_above_threshold(int8_t arr[10]) {
    uint8_t i;
    for (i = 0; i < 10; i++) {
        if (arr[i] < V_CHARGING_THRESHOLD)
            return false;
    }
    return true;
}

bool every_below_threshold(int8_t arr[10]) {
    uint8_t i;
    for (i = 0; i < 10; i++) {
        if (arr[i] >= V_CHARGING_THRESHOLD)
            return false;
    }
    return true;
}

uint16_t adc_read() {
    // start conversion
    ADCSRA |= _BV(ADSC);
    // select PB4 (ADC2) analog channel with AVCC ref
    ADMUX = _BV(MUX1);
    // wait for end of conversion
    while (ADCSRA & _BV(ADSC));
    return ADC;
}

void process_adc_read(const uint16_t adc_result_val) {
    adc_processing = true;
    uint8_t with_dot;
    if (adc_result_val >= 1000){
        digit_reads[array_counter] = 5;
        with_dot = 0;
    } else if (adc_result_val >= 921) {
        digit_reads[array_counter] = 4;
        with_dot = 1;
    } else if (adc_result_val >= 820) {
        digit_reads[array_counter] = 4;
        with_dot = 0;
    } else if (adc_result_val >= 716) {
        digit_reads[array_counter] = 3;
        with_dot = 1;
    } else if (adc_result_val >= 615) {
        digit_reads[array_counter] = 3;
        with_dot = 0;
    } else if (adc_result_val >= 512) {
        digit_reads[array_counter] = 2;
        with_dot = 1;
    } else if (adc_result_val >= 410) {
        digit_reads[array_counter] = 2;
        with_dot = 0;
    } else if (adc_result_val >= 307) {
        digit_reads[array_counter] = 1;
        with_dot = 1;
    } else if (adc_result_val >= 205) {
        digit_reads[array_counter] = 1;
        with_dot = 0;
    } else if (adc_result_val >= 102) {
        digit_reads[array_counter] = 0;
        with_dot = 1;
    } else {
        digit_reads[array_counter] = 0;
        with_dot = 0;
    }

    if (digit_reads[array_counter] != last_digit || with_dot != last_with_dot) {
        display_number(digit_reads[array_counter], with_dot);
        last_digit = digit_reads[array_counter];
        last_with_dot = with_dot;
    }

    if (array_counter == 9) {
        if (!wait) {
            if (!is_charging_on() && every_above_threshold(digit_reads)) {
                charging_on();
            } else if (is_charging_on() && every_below_threshold(digit_reads)) {
                charging_off();
                wait = true;
                wait_counter = 0;
            }
        } else {
            if (wait_counter == 10) {
                wait = false;
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
