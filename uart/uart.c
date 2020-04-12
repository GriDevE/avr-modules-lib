/*
 * uart.c
 *
 * Created: 2019.07.12 - 2020.04.12
 *  Author: GriDev
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../macros.h"
#include "uart.h"


//////// Variables

void (*GI_callback_receive_byte)(uint8_t ucsra, uint8_t frame_data) = NULL;
uint8_t (*GI_callback_transmit_byte)(void) = NULL;

void (*GI_callback_transfer_complete)(void) = NULL;


//////// Interrupts

ISR(USART_RX_vect)
{
    uint8_t ucsra = UCSR0A;
    GI_callback_receive_byte(ucsra, UDR0);
}

ISR(USART_UDRE_vect)
{
    UDR0 = GI_callback_transmit_byte();
    UCSR0A |= (1<<TXC0);
}

ISR(USART_TX_vect)
{
    UART_TX_COMPLETE_CALL_OFF;
    GI_callback_transfer_complete();
}


//////// Functions

void uart_init(void (*callback_receive_byte)(uint8_t ucsra, uint8_t frame_data),
               uint8_t (*callback_transmit_byte)(void) )
{
    UCSR0B =
        0<<RXCIE0 | // RX Complete Interrupt Enable.
        // Включает прерывания по флагу RXC0.
        0<<TXCIE0 | // TX Complete Interrupt Enable.
        // Включает прерывания по флагу TXC0.
        0<<UDRIE0 | // USART Data Register Empty Interrupt Enable.
        // Включает прерывания по флагу UDRE0.
        0<<RXEN0 | // Включает приёмник USART.
        // Отключение приемника приведет к сбросу буфера приема,
        // сбросу флагов: FE0, DOR0 и UPE0.
        0<<TXEN0 | // Включает передатчик USART.
        // Отключение передатчика (запись TXEN0 в ноль) не вступит в силу,
        // пока не будут завершены текущие и ожидающие передачи,
        // то есть пока регистр сдвига передачи и регистр буфера передачи
        // содержат данне для передачи.
        0<<UCSZ02 |
        // UCSZn2 UCSZn1 UCSZn0 Character Size
        // 0      0      0      5-bit
        // 0      0      1      6-bit
        // 0      1      0      7-bit
        // 0      1      1      8-bit
        // 1      0      0      Reserved
        // 1      0      1      Reserved
        // 1      1      0      Reserved
        // 1      1      1      9-bit
        0<<RXB80 | // Принятый 9-й бит данных.
        // Должен быть прочитан до чтения младших битов в UDR0.
        0<<TXB80; // Передаваемый 9-й бит данных.
        // Должен быть записан до записи младших битов в UDR0.
    UCSR0A =
        0<<RXC0 | // USART Receive Complete.
        // Установлен когда в приёмном буфере есть непрочитанные данные.
        // Если приемник отключен, буфер приема будет сброшен и, следовательно,
        // бит RXC0 будет в нулевом состоянии.
        // Бит может генерировать прерывание завершения приёма кадра.
        1<<TXC0 | // USART Transmit Complete.
        // Устанавливается, когда весь кадр в регистре сдвига передачи сдвинут,
        // и в буфере передачи UDR0 нет новых данных.
        // Бит может генерировать прерывание по завершению передачи кадров.
        // Бит TXC0 сбрасывается при выполнении прерывания завершения передачи,
        // также может быть сброшен записью единички.
        // Не забываем сбрасывать TXC0 перед каждой передачей (записью UDR0),
        // если мы его используем для проверки того что передача завершена.
        0<<UDRE0 | // USART Data Register Empty.
        // Указывает что буфер передачи UDR0 готов к приёму новых данных.
        // Данные записанные в UDR0 когда флаг UDRE0 не установлен будут
        // игнорироваться передатчиком.
        // Бит устанавливается после сброса контроллера.
        // Бит можно обнулить только записью в регистр UDR0.
        0<<FE0 | // Frame Error.
        // Ошибка кадра при приёме.
        // Устанавливается, когда первый стоп-бит следующего кадра
        // в приемном буфере равен нулю.
        // Бит сбрасывается когда мы читаем приёмный буфер UDR0.
        // Нужно всегда устанавливать этот бит в 0 при записи в UCSR0A.
        0<<DOR0 | // Data OverRun.
        // Устанавливается если возникло переполнение приёмного буфера,
        // уже принято 2 кадра которые мы не прочитали и поступил
        // старт-бит следующего кадра.
        // Бит сбрасывается когда мы читаем приёмный буфер UDR0.
        // Нужно всегда устанавливать этот бит в 0 при записи в UCSR0A.
        0<<UPE0 | // USART Parity Error.
        // Если включён бит паритета, то этот бит отображает ошибку паритета.
        // Нужно всегда устанавливать этот бит в 0 при записи в UCSR0A.
        0<<U2X0 | // Double the USART Transmission Speed.
        // Бит влияет только на асинхронный режим.
        // Установка в 1 удвоит скорость передачи.
        0<<MPCM0; // Multi-processor Communication Mode.
    UCSR0C =
        0<<UMSEL01 |
        0<<UMSEL00 |
        // UMSELn1 UMSELn0 Mode
        // 0       0       Asynchronous USART
        // 0       1       Synchronous USART
        // 1       0       (Reserved)
        // 1       1       Master SPI
        0<<UPM01 |
        0<<UPM00 |
        // UPMn1 UPMn0 Parity Mode
        // 0     0     Disabled
        // 0     1     Reserved
        // 1     0     Enabled, Even Parity
        // 1     1     Enabled, Odd Parity
        0<<USBS0 |
        // USBSn Stop Bit(s)
        // 0     1-bit
        // 1     2-bit
        1<<UCSZ01 |
        1<<UCSZ00 |
        0<<UCPOL0; // Используется только для синхронного режима. 
        // Должен быть записан в ноль при использовании асинхронного режима.

    GI_callback_receive_byte = callback_receive_byte;
    GI_callback_transmit_byte = callback_transmit_byte;

    // RS485
    PORT_UART_DIRECTION      cb (1<<PIN_UART_DIRECTION);
    DDR(PORT_UART_DIRECTION) sb (1<<PIN_UART_DIRECTION);
        // Компилируется в cbi, sbi - поэтому можно не отключать прерывания.
}

void uart_set_baud(uint16_t baud)
{
    uint8_t sreg = SREG;
    cli();
    UBRR0 = (F_CPU/16)/baud - 1;
    SREG = sreg;
}

void set_callback_uart_tx_complete(void (*callback_transfer_complete)(void) )
{
    uint8_t sreg = SREG;
    cli();
    GI_callback_transfer_complete = callback_transfer_complete;
    SREG = sreg;
}
