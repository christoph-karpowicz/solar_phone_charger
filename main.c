#include <avr/wdt.h>
#include <util/delay.h>
#include "display.h"

void init() {
    // init display
    DDRB |= _BV(SHIFT_REG_SCK_PIN) | _BV(SHIFT_REG_RCK_PIN) | _BV(SHIFT_REG_SERIAL_PIN);

    // disable watchdog
    wdt_disable();

    PORTB |= _BV(PB3);
}

int main(void) {
    init();
    _delay_ms(1000);
    uint8_t i = 0;
    display_number(8);
    while (1) {
        _delay_ms(1000);
        if (i % 2 == 0) {
            PORTB |= _BV(PB3);
        } else {
            PORTB &= ~_BV(PB3);
        }
        i++;
    }
    return 0;
}