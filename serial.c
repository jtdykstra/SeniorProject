// Authors: Robert Prosser, Jordan Dykstra
#include <avr/io.h>
#include <avr/interrupt.h>
#include "globals.h"

/*
 * Initialize the serial port.
 */
void serial_init() {
   uint16_t baud_setting;

   UCSR0A = _BV(U2X0);
   baud_setting = 16; //115200 baud

   // assign the baud_setting
   UBRR0H = baud_setting >> 8;
   UBRR0L = baud_setting;

   // enable transmit and receive
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
}

/*
 * Return 1 if a character is available else return 0.
 */
uint8_t byte_available() {
   return (UCSR0A & (1 << RXC0)) ? 1 : 0;
}

/*
 * Unbuffered read
 * Return 255 if no character is available otherwise return available character.
 */
uint8_t read_byte() {
   if (UCSR0A & (1 << RXC0)) return UDR0;
   return 255;
}

/*
 * Unbuffered write
 *
 * b byte to write.
 */
uint8_t write_byte(uint8_t b) {
   //loop until the send buffer is empty
   while (((1 << UDRIE0) & UCSR0B) || !(UCSR0A & (1 << UDRE0))) {}

   //write out the byte
   UDR0 = b;
   return 1;
}

void print_string(char *s) {
//   cli();
   while (*s) {
      write_byte(*s);
      s++;   
   }
  // sei();
}

void print_int(uint16_t i) {
   uint16_t div = 10000;
   uint8_t remainder;
   remainder = i / div;
   if (i == 0) {
      write_byte('0');
      return;
   }
   
   while (remainder == 0 && div != 1) {
      div /= 10;
      remainder = i / div;
   }
   while (div != 0) {
      write_byte(remainder + NUM_START);
      i -= remainder * div;
      div /= 10;
      remainder = i / div;
   }
}

void print_int32(uint32_t i) {
   uint32_t div = 1000000000;
   uint8_t remainder;
   remainder = i / div;
   while (remainder == 0) {
      div /= 10;
      remainder = i / div;
   }
   while (div != 0) {
      write_byte(remainder + NUM_START);
      i -= remainder * div;
      div /= 10;
      remainder = i / div;
   }
}

void print_hex(uint16_t i) {
   write_byte(48);
   write_byte(120);
   uint16_t mask = 15;
   int8_t shift = 3;
   uint16_t temp;
   while (shift >= 0) {
      temp = ((mask << shift * 4) & i) >> shift * 4;
      if (temp > 9) {
         write_byte(temp + 55);
      }
      else {
         write_byte(temp + NUM_START);
      }
      shift--;
   }

}

void print_hex32(uint32_t i) {
   write_byte(48);
   write_byte(120);
   uint32_t mask = 15;
   int8_t shift = 7;
   uint16_t temp;
   while (shift >= 0) {
      temp = ((mask << shift * 4) & i) >> shift * 4;
      if (temp > 9) {
         write_byte(temp + 55);
      }
      else {
         write_byte(temp + NUM_START);
      }
      shift--;
   }
}

void set_cursor(uint8_t row, uint8_t col) {
   write_byte(ESC);
   write_byte(SQBRACKET);
   print_int(row);
   write_byte(';');
   print_int(col);
   write_byte('f');
}

void set_color(uint8_t color) {
   write_byte(ESC);
   write_byte(SQBRACKET);
   print_int(color);
   write_byte('m');
}

void clear_screen(void) {
   write_byte(ESC);
   write_byte(SQBRACKET);
   write_byte('2');
   write_byte('J');
}

