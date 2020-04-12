/*
 * uart.h
 *
 * Created: 2019.07.12 - 2020.04.12
 *  Author: GriDev
 */

#pragma once

//////// Задействованные ресурсы
//
// USART0
// PORT_UART
// PORT_UART_DIRECTION
//
// Прерывания:
//   USART_RX_vect
//   USART_UDRE_vect
//   USART_TX_vect
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

#define PORT_UART PORTD

// RX <-
#define PIN_UART_RX PD0

// TX ->
#define PIN_UART_TX PD1

// IO_SELECTION ->
#define PORT_UART_DIRECTION PORTD
#define PIN_UART_DIRECTION PD5
#define RS485_IN  PORT_UART_DIRECTION cb (1<<PIN_UART_DIRECTION)
#define RS485_OUT PORT_UART_DIRECTION sb (1<<PIN_UART_DIRECTION)
#define RS485_STATE_IN (!(PORT_UART_DIRECTION & 1<<PIN_UART_DIRECTION))
#define RS485_STATE_OUT (PORT_UART_DIRECTION & 1<<PIN_UART_DIRECTION)


//////// Definitions

// Ошибка в принятом кадре
#define UART_RECEIVE_FRAME_ERROR(ucsra) (ucsra & 1<<FE0)
// Переполнился буфер приёма, не успели прочитать
#define UART_RECEIVE_OVERFLOW(ucsra) (ucsra & 1<<DOR0)


//////// Functions

// Инициализирует модуль UART
void uart_init(void (*callback_receive_frame)(uint8_t errors, uint8_t frame),
               uint8_t (*callback_transmit_frame)(void) );
    // Коллбэки будут вызываться из прерываний!

void uart_set_baud(uint16_t baud);
    // Перед вызовом убедиться, что передача завершена!

// Включить/выключить приёмник и прерывание UART по завершению приёма кадра
#define UART_RECEIVE_ON  UCSR0B |=  (1<<RXEN0 | 1<<RXCIE0)
#define UART_RECEIVE_OFF UCSR0B &= ~(1<<RXEN0 | 1<<RXCIE0)
    // Прерывание вызывает callback_receive_frame(frame),
    // в который передаётся принятый кадр.

// Включить/выключить передатчик и прерывание UART по готовности UDR0 для записи
#define UART_TRANSFER_ON  UCSR0B |=  (1<<TXEN0 | 1<<UDRIE0)
    // Прерывание вызывает callback_transmit_frame(),
    // который передаёт следующий кадр.
#define UART_TRANSFER_I_OFF UCSR0B &= ~(1<<UDRIE0) // Отключает прерывание.
#define UART_TRANSFER_OFF UCSR0B &= ~(1<<TXEN0) // Отключает передатчик.

// Устанавливает коллбэк вызываемый по завершению передачи кадров
void uart_set_callback_tx_complete(void (*callback_transfer_complete)(void) );
    // Коллбэк будет вызываться из прерывания!

// Включить/выключить вызов коллбэка по завершению передачи кадров
#define UART_TX_COMPLETE_CALL_ON  UCSR0B |=  (1<<TXCIE0)
    // Коллбэк будет вызван только 1 раз, потом нужно будет включать снова.
#define UART_TX_COMPLETE_CALL_OFF UCSR0B &= ~(1<<TXCIE0)
