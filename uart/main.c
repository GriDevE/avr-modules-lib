/*
 * main.h
 *
 * Created: 2020.04.11 - 2020.04.12
 *  Author: GriDev
 */

#include <avr/interrupt.h>

#include "uart.h"
#include "../macros.h"
#include "main.h"


//////// Variables

uint8_t G_buffer_tx[10] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
uint8_t G_n_tx = 10;

uint8_t G_buffer_rx[10];
uint8_t G_n_rx = 0;


//////// Private Functions

//// Callback functions from UART ISR

void receive_frame(uint8_t errors, uint8_t frame);
uint8_t transmit_frame(void);
void callback_transfer_complete(void);


////////

int main(void)
{
    sei();
    uart_init(receive_frame, transmit_frame);
    uart_set_baud(UART_SPEED);
    uart_set_callback_tx_complete(callback_transfer_complete);

    UART_RECEIVE_OFF;
    RS485_OUT;
    UART_TRANSFER_ON;

    for ( ; ; ) {
        volatile uint8_t* n_rx_p = &G_n_rx;
            // Нам нужен именно указатель на volatile переменную,
            // чтобы компилятор корректно применил оптимизацию.
        if (*n_rx_p > 0) {
            if (G_buffer_rx[0] == 0xAA) {
                // Как получим 0xAA, сразу начинаем отправляем 1, 2, 3.

                UART_RECEIVE_OFF;

                G_n_tx = 3;
                G_buffer_tx[2] = 1;
                G_buffer_tx[1] = 2;
                G_buffer_tx[0] = 3;
                
                RS485_OUT;
                UART_TRANSFER_ON;
            }
            G_n_rx = 0;
        }
    }
}

void receive_frame(uint8_t errors, uint8_t frame)
{
    if (G_n_rx < sizeof G_buffer_rx) {
        G_buffer_rx[G_n_rx] = frame;
        G_n_rx++;
        if (UART_RECEIVE_FRAME_ERROR(errors)) {
            // Ошибка приёма кадра, нулевой стоп-бит
        }
        if (UART_RECEIVE_OVERFLOW(errors)) {
            // Ошибка, переполнение буфера приёма
        }
    } else {
        // Переполнение нашего буфера
    }
}

uint8_t transmit_frame(void)
{
    G_n_tx--;
    if (G_n_tx == 0) {
        UART_TRANSFER_I_OFF;
        UART_TX_COMPLETE_CALL_ON;
    }
    return G_buffer_tx[G_n_tx];
}

void callback_transfer_complete(void)
{
    RS485_IN;
    UART_TRANSFER_OFF;
    UART_RECEIVE_ON;
}
