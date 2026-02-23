#include "display.h"

#define OUTPUT_PINS 8

static const uint8_t DIGITS[] PROGMEM = {
    0b11101101, // 0
    0b10000100, // 1
    0b11011001, // 2
    0b11011100, // 3
    0b10110100, // 4
    0b01111100, // 5
    0b00000000 // empty
};

void display_number(const int8_t digit, const int8_t with_dot) {
    display(pgm_read_word(&DIGITS[digit]) | ((1 << with_dot) & 2));
}

void display_empty() {
    display(DIGITS[6]);
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