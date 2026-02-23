#ifndef DISPLAY_H
#define DISPLAY_H

#include <avr/io.h>
#include <avr/pgmspace.h>

#define SHIFT_REG_PORT PORTB
#define SHIFT_REG_SERIAL_PIN PB0
#define SHIFT_REG_SCK_PIN PB2
#define SHIFT_REG_RCK_PIN PB1

/*
    7
   ___ 
  |   |
6 |_5_| 8
  |   |
1 |___| 3  .2
    4
*/

void display_number(const int8_t digit, const int8_t with_dot);
void display_empty();
void display(const uint8_t output);

#endif