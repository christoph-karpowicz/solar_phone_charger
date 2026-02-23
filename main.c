// #define F_CPU 1200000
#define F_CPU 37500

#include <util/delay.h>
#include <avr/interrupt.h>
#include "display.h"

#define V_CHARGING_THRESHOLD 4

volatile int8_t last_digit;
volatile int8_t last_with_dot;
volatile uint16_t blink_counter = 0;
volatile int8_t digit_reads[10];
volatile int8_t dot_reads[10];
volatile uint8_t read_counter = 0;

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
    ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE);
    // set ADC auto trigger source to Timer/Counter Compare Match B
    ADCSRB |= _BV(ADTS2) | _BV(ADTS0);
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
    // CLKPR = _BV(CLKPS1) | _BV(CLKPS0);

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

int8_t find_max(volatile int8_t arr[10]) {
    int8_t max = arr[0];
    int8_t i;
    for (i = 1; i < 10; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

// ADC Conversion Complete interrupt
ISR(ADC_vect) {
    uint16_t adc_result = ADC;
    if (adc_result >= 1000){
        digit_reads[read_counter] = 5;
        dot_reads[read_counter] = 0;
    } else if (adc_result >= 921) {
        digit_reads[read_counter] = 4;
        dot_reads[read_counter] = 1;
    } else if (adc_result >= 820) {
        digit_reads[read_counter] = 4;
        dot_reads[read_counter] = 0;
    } else if (adc_result >= 716) {
        digit_reads[read_counter] = 3;
        dot_reads[read_counter] = 1;
    } else if (adc_result >= 615) {
        digit_reads[read_counter] = 3;
        dot_reads[read_counter] = 0;
    } else if (adc_result >= 512) {
        digit_reads[read_counter] = 2;
        dot_reads[read_counter] = 1;
    } else if (adc_result >= 410) {
        digit_reads[read_counter] = 2;
        dot_reads[read_counter] = 0;
    } else if (adc_result >= 307) {
        digit_reads[read_counter] = 1;
        dot_reads[read_counter] = 1;
    } else if (adc_result >= 205) {
        digit_reads[read_counter] = 1;
        dot_reads[read_counter] = 0;
    } else if (adc_result >= 102) {
        digit_reads[read_counter] = 0;
        dot_reads[read_counter] = 1;
    } else {
        digit_reads[read_counter] = 0;
        dot_reads[read_counter] = 0;
    }

    if (read_counter == 9) {
        display_empty();
        int8_t max_digit_read = find_max(digit_reads);
        int8_t max_dot_read = find_max(dot_reads);
        if (max_digit_read != last_digit || max_dot_read != last_with_dot) {
            if (max_digit_read < V_CHARGING_THRESHOLD) {
                charging_off();
            } else {
                charging_on();
                blink_counter = 0;
            }
            display_number(max_digit_read, max_dot_read);
            last_digit = max_digit_read;
            last_with_dot = max_dot_read;
        }
        read_counter = 0;
    }

    blink_counter++;
    read_counter++;
}

int main(void) {
    init();
    while (1);
    return 0;
}
