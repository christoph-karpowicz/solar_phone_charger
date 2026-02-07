#define F_CPU 9600000

#include <avr/io.h>
#include <util/delay.h>
#include "display.h"

void init() {
    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);

    // enable ADC with 64 prescaler division factor
    ADCSRA |= _BV(ADPS1) | _BV(ADPS2) | _BV(ADEN);
    // disable ADC
    // ADCSRA &= ~_BV(ADEN);

    // PORTB |= _BV(PB3);
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

int main(void) {
    init();
    while (1) {
        _delay_ms(200);
        uint16_t adc_result = adc_read();
        if (adc_result >= 1000){
            display_number(5);
        } else if (adc_result >= 820) {
            display_number(4);
        } else if (adc_result >= 615) {
            display_number(3);
        } else if (adc_result >= 410) {
            display_number(2);
        } else if (adc_result >= 205) {
            display_number(1);
        } else {
            display_number(0);
        }
    }
    return 0;
}