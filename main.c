#define F_CPU 9600000

#include <util/delay.h>
#include <avr/interrupt.h>
#include "display.h"

struct DisplayNumber last_dn;

void init() {
    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);
    // init driver
    DDRB |= _BV(PB3);

    // Timer/Counter Output Compare Match B Interrupt Enable
    TIMSK0 |= _BV(OCIE0B);
    // set the timer's TOP value
    OCR0B = 1024;
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

    // Enable global interrupts
    sei();

    // set to out of bounds numbers
    last_dn.digit = 9;
    last_dn.with_dot = 9;
}

void charging_on() {
    PORTB |= _BV(PB3);
}

void charging_off() {
    PORTB &= ~_BV(PB3);
}

// ADC Conversion Complete interrupt
ISR(ADC_vect) {
    uint16_t adc_result = ADC;
    struct DisplayNumber dn;
    if (adc_result >= 1000){
        dn.digit = 5;
        dn.with_dot = 0;
    } else if (adc_result >= 921) {
        dn.digit = 4;
        dn.with_dot = 1;
    } else if (adc_result >= 820) {
        dn.digit = 4;
        dn.with_dot = 0;
    } else if (adc_result >= 716) {
        dn.digit = 3;
        dn.with_dot = 1;
    } else if (adc_result >= 615) {
        dn.digit = 3;
        dn.with_dot = 0;
    } else if (adc_result >= 512) {
        dn.digit = 2;
        dn.with_dot = 1;
    } else if (adc_result >= 410) {
        dn.digit = 2;
        dn.with_dot = 0;
    } else if (adc_result >= 307) {
        dn.digit = 1;
        dn.with_dot = 1;
    } else if (adc_result >= 205) {
        dn.digit = 1;
        dn.with_dot = 0;
    } else if (adc_result >= 102) {
        dn.digit = 0;
        dn.with_dot = 1;
    } else {
        dn.digit = 0;
        dn.with_dot = 0;
    }

    if (dn.digit != last_dn.digit || dn.with_dot != last_dn.with_dot) {
        display_number(dn);
        if (dn.digit < 4) {
            charging_off();
        } else {
            charging_on();
        }
        last_dn = dn;
    }
}

int main(void) {
    init();
    while (1);
    return 0;
}
