#define F_CPU 9600000

#include <util/delay.h>
#include "display.h"

void init() {
    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);
    // init driver
    DDRB |= _BV(PB3);

    // enable ADC with 64 prescaler division factor
    ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN);
    // disable ADC
    // ADCSRA &= ~_BV(ADEN);
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

void charging_on() {
    PORTB |= _BV(PB3);
}

void charging_off() {
    PORTB &= ~_BV(PB3);
}

int main(void) {
    init();
    while (1) {
        _delay_ms(200);
        uint16_t adc_result = adc_read();
        if (adc_result >= 1000){
            display_number(5, 0);
            charging_on();
        } else if (adc_result >= 921) {
            display_number(4, 1);
            charging_on();
        } else if (adc_result >= 820) {
            display_number(4, 0);
            charging_on();
        } else if (adc_result >= 716) {
            display_number(3, 1);
            charging_off();
        } else if (adc_result >= 615) {
            display_number(3, 0);
            charging_off();
        } else if (adc_result >= 512) {
            display_number(2, 1);
            charging_off();
        } else if (adc_result >= 410) {
            display_number(2, 0);
            charging_off();
        } else if (adc_result >= 307) {
            display_number(1, 1);
            charging_off();
        } else if (adc_result >= 205) {
            display_number(1, 0);
            charging_off();
        } else if (adc_result >= 102) {
            display_number(0, 1);
            charging_off();
        } else {
            display_number(0, 0);
            charging_off();
        }
    }
    return 0;
}