/*
 * sw_uart.h
 *
 * Created: 2019.09.19 - 2020.04.12
 *  Author: GriDev
 */

#pragma once

//////// Задействованные ресурсы
//
// TIMER1
// PORT_SWUART
// PORT_SWUART_DIRECTION
//
// Прерывания:
//   INT0_vect
//   TIMER1_COMPA_vect
//   TIMER1_COMPB_vect
//
//////// Методика портирования модуля
//
// * Настроить PIN_UART_DIRECTION.
//
// * Отредактировать необходимые настройки USART в функции uart_init().
//   Проанализировать код модуля на необходимость изменений под новые настройки.
//
// * Если мы меняем UCSR0B из прерываний,
//   внимательней с макросами меняющими UCSR0B.
//
////////

//////// Ports

#define PORT_SWUART PORTD

// RX <-
#define PIN_SWUART_RX PD2 // INT0 - used

// TX ->
#define PIN_SWUART_TX PD4

// IO_SELECTION ->
#define PORT_SWUART_DIRECTION PORTD
#define PIN_SWUART_DIRECTION PD3 // INT1 - not used
#define SW_RS485_IN  PORT_SWUART_DIRECTION cb (1<<PIN_SWUART_DIRECTION)
#define SW_RS485_OUT PORT_SWUART_DIRECTION sb (1<<PIN_SWUART_DIRECTION)
#define SW_RS485_STATE (PORT_SWUART_DIRECTION & 1<<PIN_SWUART_DIRECTION)


//////// Definitions

// Ошибка в принятом кадре, нулевой стоп-бит
enum swuart_errors_t {
    SWUART_ERROR_RECEIVE_FRAME = 1<<0,
};

// Приватные флаги, не трогать.
// Вынесены в h-файл поскольку используются в макросах.
enum swuart_flags_t {
    F_TRANSFER_ON = 1<<0, // Флаг включения/отключения вызова коллбэка
        // callback_transmit_frame(), вызывается когда готовы
        // записать в буфер следующий кадр для передачи.
        // Аналог бита UDRIE в железном USART.
    F_TRANSFER_NEXT_FRAME = 1<<1,
        // Внутренняя команда на передачу следующего кадра.
    F_TX_COMPLETE_CALL_ON = 1<<2,
        // Команда на вызов коллбэка по завершению передачи кадра.
};


//////// Functions

// Инициализирует модуль SW UART
void swuart_init(void (*callback_receive_frame)(uint8_t errors, uint8_t frame), 
                 uint8_t (*callback_transmit_frame)(void) );
    // Коллбэки будут вызываться из прерываний!

void swuart_set_baud(uint16_t baud);
 // Перед использованием убедиться, что передача/приём завершена!

// Включает приём кадров
void swuart_receive_on(void); 
// Отключает приём кадров
void swuart_receive_off(void);
    // Если вызвана во время приёма кадра,
    // прервёт приём, очистит буфер приёма.

// Включает передачу кадров и вызов коллбэка callback_transmit_frame()
void swuart_transfer_on(void);
    // Вызванный коллбэк callback_transmit_frame(), передаёт следующий кадр.
// Выключает передачу кадров и вызов коллбэка callback_transmit_frame()
void swuart_transfer_off(void);
    // Если в момент вызова есть не переданный кадр, он успешно отправится,
    // после чего передача будет выключена.

// Устанавливает коллбэк вызываемый по завершению передачи кадра
void swuart_set_callback_tx_complete(void (*callback_transfer_complete)(void) );
    // Коллбэк будет вызываться из прерывания!

// Включить/выключить вызов коллбэка по завершению передачи кадров
void swuart_tx_complete_call_on(void);
    // Коллбэк будет вызван только 1 раз, потом нужно будет включать снова.
void swuart_tx_complete_call_off(void);


//// Дублирующий функционал в виде макросов.
// Выполнять только с отключенными прерываниями!

#define SWUART_RECEIVE_ON do {   \
        EIFR |= 1<<INTF0;        \
        EIMSK |= 1<<INT0;        \
    } while(0)
    // Можно вызывать только, если точно знаем что приём отключен!

#define SWUART_RECEIVE_OFF do {    \
        TIMSK1 &= ~(1<<OCIE1A);    \
        EIMSK &= ~(1<<INT0);       \
        if (TIMSK1 & 1<<OCIE1B) {  \
            TCCR1B = 0;            \
        }                          \
    } while(0)

#define SWUART_TRANSFER_OFF do {              \
        extern uint8_t GI_swuart_flags;       \
        GI_swuart_flags &= ~(F_TRANSFER_ON);  \
    } while(0)

#define SWUART_TX_COMPLETE_CALL_ON do {            \
        extern uint8_t GI_swuart_flags;            \
        GI_swuart_flags |= F_TX_COMPLETE_CALL_ON;  \
    } while(0)

#define SWUART_TX_COMPLETE_CALL_OFF do {              \
        extern uint8_t GI_swuart_flags;               \
        GI_swuart_flags &= ~(F_TX_COMPLETE_CALL_ON);  \
    } while(0)
