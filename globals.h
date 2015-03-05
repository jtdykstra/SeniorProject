/*  Authors: Robert Prosser, Jordan Dysktra
 */
//Global defines
#include <avr/io.h>

#ifndef GLOBALS_H
#define GLOBALS_H

//place defines and prototypes here
void print_string(char* s);
void print_int(uint16_t i);
void print_int32(uint32_t i);
void print_hex(uint16_t i);
void print_hex32(uint32_t i);
void set_cursor(uint8_t row, uint8_t col);
void set_color(uint8_t color);
void clear_screen(void);
int main(void);

#define RED 31
#define GREEN 32
#define YELLOW 33
#define BLUE 34
#define MAGENTA 35
#define CYAN 36
#define WHITE 37
#define ESC 27
#define SQBRACKET 91
#define NUM_START 48

#endif
