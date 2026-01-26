#include "display.h"

#define OUTPUT_PINS 8

static const uint8_t DIGITS[] PROGMEM = {
    0b11101101,
    0b10000100,
    0b11011001,
    0b11011100,
    0b10110100,
    0b01111100,
    0b00111101,
    0b11000100,
    0b11111101,
    0b11101100
};

void display_number(const uint8_t number) {
    display(pgm_read_word(&DIGITS[number]));
}

void display(const uint8_t output) {
    SHIFT_REG_PORT &= ~_BV(SHIFT_REG_RCK_PIN);
    uint8_t i;
    uint8_t mask = 0b10000000;
    for (i = OUTPUT_PINS; i > 0; i--) {
        SHIFT_REG_PORT &= ~_BV(SHIFT_REG_SCK_PIN);
        if (output & mask) {
            SHIFT_REG_PORT |= _BV(SHIFT_REG_SERIAL_PIN);
        } else {
            SHIFT_REG_PORT &= ~_BV(SHIFT_REG_SERIAL_PIN);
        }
        SHIFT_REG_PORT |= _BV(SHIFT_REG_SCK_PIN);
        mask >>= 1;
    }
    SHIFT_REG_PORT |= _BV(SHIFT_REG_RCK_PIN);
}