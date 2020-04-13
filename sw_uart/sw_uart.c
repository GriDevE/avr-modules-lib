/*
 * sw_uart.c
 *
 * Created: 2019.09.19 - 2020.04.12
 *  Author: GriDev
 */

#include <stdlib.h>

#include <avr/interrupt.h>

#include "../macros.h"
#include "sw_uart.h"

//// LED
#define PORT_LED PORTB
// LED Red
#define PIN_LED_R PB1
#define LED_R_ON  DDR(PORT_LED) sb (1<<PIN_LED_R)
#define LED_R_OFF DDR(PORT_LED) cb (1<<PIN_LED_R)
#define LED_R_INV DDR(PORT_LED) ^= (1<<PIN_LED_R)

//////// Variables

uint8_t GI_swuart_flags = 0;

// Переменные для приёма кадра
struct swuart_rx_t {
    uint16_t time_one_bit; // Длительность одного бита.
    uint8_t byte; // Принимаемый кадр.
    uint8_t counter_bit; // Счётчик принятых бит кадра.
    void (*I_callback_receive_frame)(uint8_t errors, uint8_t frame);
} G_swuart_rx;

// Переменные для передачи кадра
struct swuart_tx_t {
    uint16_t time_one_bit; // Длительность одного бита.
    uint8_t byte; // Отправляемый кадр.
    uint8_t counter_bit; // Счётчик отправленных бит кадра.
    uint8_t (*I_callback_transmit_frame)(void);
    void (*I_callback_transfer_complete)(void);
} G_swuart_tx;


//////// Definitions

#define TIMER_ON TCCR1B = (0<<CS12 | 0<<CS11 | 1<<CS10)
    // На TIMER1 цепляем CLK без делителя.
#define TIMER_OFF TCCR1B = 0


//////// Interrupts

// Ловит предполагаемый старт-бит
ISR(INT0_vect)
{
    TIMER_ON;
    OCR1A = TCNT1 + (G_swuart_rx.time_one_bit>>1);
    TIFR1 sb 1<<OCF1A;
    TIMSK1 sb 1<<OCIE1A;
    EIMSK cb (1<<INT0);

    G_swuart_rx.counter_bit = 0;
    G_swuart_rx.byte = 0;
}

// Приём бит кадра
ISR(TIMER1_COMPA_vect)
{
    uint8_t i = G_swuart_rx.counter_bit;

    if (i == 0) {
        // Читаем старт-бит
        if ( PIN(PORT_SWUART) & (1<<PIN_SWUART_RX) ) {
            // Хрень прилетела - снова ловим старт бит.
            EIFR sb 1<<INTF0;
            EIMSK sb 1<<INT0;
            TIMSK1 cb (1<<OCIE1A);
            // Потенциально нехорошее поведение алгоритма в случае если проблемы
            // с линией RX и будет поступать шум на линии, то он может постоянно
            // начать вызывать прерывание int что может привести к зависанию
            // основной программы.
            // Если есть риск возникновения такой ситуации то нужно доработать
            // код, введя задержку, не сразу включая прерывание int.
            return;
        } else {
            OCR1A += G_swuart_rx.time_one_bit;
            TIFR1 sb 1<<OCF1A;
            i = 1;
        }
    } else if (i < 9) {
        // Читаем биты данных
        OCR1A += G_swuart_rx.time_one_bit;
        TIFR1 sb 1<<OCF1A;
        uint8_t byte = G_swuart_rx.byte;
        byte >>= 1;
        if ( PIN(PORT_SWUART) & 1<<PIN_SWUART_RX ) {
            byte |= 1<<7;
        }
        G_swuart_rx.byte = byte;
        i++;
    } else {
        // Читаем стоп-бит
        if ( PIN(PORT_SWUART) & 1<<PIN_SWUART_RX ) {
            G_swuart_rx.I_callback_receive_frame(0, G_swuart_rx.byte);
        } else {
            G_swuart_rx.I_callback_receive_frame(SWUART_ERROR_RECEIVE_FRAME,
                                                 G_swuart_rx.byte);
        }
        // Инициируем ожидание следующего кадра
        TIMSK1 cb (1<<OCIE1A);
        EIFR sb 1<<INTF0;
        EIMSK sb 1<<INT0;
        //
        return;
    }

    G_swuart_rx.counter_bit = i;
}

// Передача бит кадра
ISR(TIMER1_COMPB_vect)
// При скорости 9600/8-N-1, длительность 1 кадра должна быть: 1042us.
{
    uint8_t i = G_swuart_tx.counter_bit;

    if (i == 0) {
        // Старт-бит сформировали
        OCR1B += G_swuart_tx.time_one_bit;
        TIFR1 sb 1<<OCF1B;
        i = 1;
        if (G_swuart_tx.byte & i) {
            PORT_SWUART sb 1<<PIN_SWUART_TX;
        }
    } else if (i < 1<<7) {
        // Формируем биты данных
        OCR1B += G_swuart_tx.time_one_bit;
        TIFR1 sb 1<<OCF1B;
        i <<= 1;
        if (G_swuart_tx.byte & i) {
            PORT_SWUART sb (1<<PIN_SWUART_TX);
        } else {
            PORT_SWUART cb (1<<PIN_SWUART_TX);
        }
    } else if (i == 1<<7) {
        // Формируем стоп-бит
        OCR1B += G_swuart_tx.time_one_bit;
        TIFR1 sb 1<<OCF1B;
        i++; // i == 0b10000001;
        PORT_SWUART sb (1<<PIN_SWUART_TX);
        // Выполняем коллбэк получения нового кадра
        if (GI_swuart_flags & F_TRANSFER_ON) {
            G_swuart_tx.byte = G_swuart_tx.I_callback_transmit_frame();
            GI_swuart_flags sb F_TRANSFER_NEXT_FRAME;
        }
    } else {
        // Формируем старт-бит следующего кадра
        if (GI_swuart_flags & F_TRANSFER_NEXT_FRAME) {
            GI_swuart_flags cb (F_TRANSFER_NEXT_FRAME);
            i = 0;
            PORT_SWUART cb (1<<PIN_SWUART_TX);
            OCR1B = TCNT1 + G_swuart_tx.time_one_bit;
        } else {
            TIMSK1 cb (1<<OCIE1B);
            if (!( TIMSK1 & 1<<OCIE1A )) {
                TIMER_OFF;
            }
            // Выполняем коллбэк по завершению передачи кадра
            if (GI_swuart_flags & F_TX_COMPLETE_CALL_ON) {
                GI_swuart_flags cb (F_TX_COMPLETE_CALL_ON);
                G_swuart_tx.I_callback_transfer_complete();
            }
            return;
        }
    }
    G_swuart_tx.counter_bit = i;
}


//////// Functions

void swuart_init(void (*callback_receive_frame)(uint8_t errors, uint8_t frame), 
                 uint8_t (*callback_transmit_frame)(void) )
{
    uint8_t sreg = SREG;
    cli();
    EIMSK cb (1<<INT0);
    TIMSK1 cb (1<<OCIE1A | 1<<OCIE1B);
    SREG = sreg;

    GI_swuart_flags = 0;

    G_swuart_rx.counter_bit = 0;
    G_swuart_rx.I_callback_receive_frame = callback_receive_frame;

    G_swuart_tx.counter_bit = 0;
    G_swuart_tx.I_callback_transmit_frame = callback_transmit_frame;
    G_swuart_tx.I_callback_transfer_complete = NULL;

    //
    DDR(PORT_SWUART) cb (1<<PIN_SWUART_RX);
    PORT_SWUART      sb (1<<PIN_SWUART_RX);
    PORT_SWUART      sb (1<<PIN_SWUART_TX);
    DDR(PORT_SWUART) sb (1<<PIN_SWUART_TX);
    // RS485
    PORT_SWUART_DIRECTION      cb (1<<PIN_SWUART_DIRECTION);
    DDR(PORT_SWUART_DIRECTION) sb (1<<PIN_SWUART_DIRECTION);
    //
    EICRA sb (1<<ISC01 | 0<<ISC00); // Прерывание по спаду INT0
    TIMER_OFF;
}

void swuart_set_baud(uint16_t baud)
{
    uint16_t time_one_bit = F_CPU / baud;

    uint8_t sreg = SREG;
    cli();
    G_swuart_rx.time_one_bit = time_one_bit;
    G_swuart_tx.time_one_bit = time_one_bit;
    SREG = sreg;
}

void swuart_receive_on(void)
{
    if (!( EIMSK & 1<<INT0 )) {
        if (!( TIMSK1 & 1<<OCIE1A )) {
            // Значит приём отключен
            EIFR sb 1<<INTF0;
            EIMSK sb 1<<INT0;
        }
    }
}
void swuart_receive_off(void)
{
    uint8_t sreg = SREG;
    cli();
    TIMSK1 cb (1<<OCIE1A);
    EIMSK cb (1<<INT0);
    SREG = sreg;
    cli(); // На случай если swuart_transfer_on() из прерывания вызовут.
    if (!( TIMSK1 & 1<<OCIE1B )) {
        TIMER_OFF;
    }
    SREG = sreg;
}

void swuart_transfer_on(void)
{
    if (!( TIMSK1 & 1<<OCIE1B )) {
        // Значит передача отключена
        G_swuart_tx.counter_bit = 0;

        uint8_t sreg = SREG;
        cli();
        GI_swuart_flags sb F_TRANSFER_ON;
        SREG = sreg;
        
        // Формируем старт-бит
        PORT_SWUART cb (1<<PIN_SWUART_TX);
        OCR1B = TCNT1 + G_swuart_tx.time_one_bit;
        
        TIFR1 sb 1<<OCF1B;
        cli();
        TIMSK1 sb 1<<OCIE1B;
        TIMER_ON;
        G_swuart_tx.byte = G_swuart_tx.I_callback_transmit_frame();
        SREG = sreg;
    }
}
void swuart_transfer_off(void)
{
    uint8_t sreg = SREG;
    cli();
    GI_swuart_flags cb (F_TRANSFER_ON);
    SREG = sreg;
}

void swuart_set_callback_tx_complete( void (*callback_transfer_complete)(void) )
{
    uint8_t sreg = SREG;
    cli();
    G_swuart_tx.I_callback_transfer_complete = callback_transfer_complete;
    SREG = sreg;
}

void swuart_tx_complete_call_on(void)
{
    uint8_t sreg = SREG;
    cli();
    GI_swuart_flags sb F_TX_COMPLETE_CALL_ON;
    SREG = sreg;
}
void swuart_tx_complete_call_off(void)
{
    uint8_t sreg = SREG;
    cli();
    GI_swuart_flags cb (F_TX_COMPLETE_CALL_ON);
    SREG = sreg;
}
