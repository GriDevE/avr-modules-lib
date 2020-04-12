#pragma once

//////// Read / Write of Ports

#define DDR(x) (*(&x - 1)) // Address of data direction register of port x
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
    // on ATmega64/128 PINF is on port 0x00 and not 0x60
    #define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
#else
    #define PIN(x) (*(&x - 2)) // Address of input register of port x
#endif

//////// Set / Clear bit

#define sb |=
#define cb(expression) &=~(expression) 
    // Используем параметр чтобы исключить потенциальные ошибки
    // связанные с приоритетом операции '~',
    // чтобы вынудить программиста поставить скобочки.

////////

#define _NOP __asm__ volatile ("nop")
