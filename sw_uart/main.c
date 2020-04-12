/*
 * main.h
 *
 * Created: 2020.04.11 - 2020.04.12
 *  Author: GriDev
 */

#include <avr/interrupt.h>

#include "sw_uart.h"
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
    swuart_init(receive_frame, transmit_frame);
    swuart_set_baud(SW_UART_SPEED);
    swuart_set_callback_tx_complete(callback_transfer_complete);

    swuart_receive_off();
    SW_RS485_OUT;
    swuart_transfer_on();

    for ( ; ; ) {
        volatile uint8_t* n_rx_p = &G_n_rx;
            // Нам нужен именно указатель на volatile переменную,
            // чтобы компилятор корректно применил оптимизацию.
        if (*n_rx_p > 0) {
            if (G_buffer_rx[0] == 0xAA) {
                // Как получим 0xAA, сразу начинаем отправляем 1, 2, 3.

                swuart_receive_off();

                G_n_tx = 3;
                G_buffer_tx[2] = 1;
                G_buffer_tx[1] = 2;
                G_buffer_tx[0] = 3;
                
                SW_RS485_OUT;
                swuart_transfer_on();
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
        if (errors & SWUART_ERROR_RECEIVE_FRAME) {
            // Ошибка приёма кадра, нулевой стоп-бит
        }
    } else {
        // Переполнение нашего буфера
    }
}

uint8_t transmit_frame(void)
{
    G_n_tx--;
    if (G_n_tx == 0) {
        SWUART_TRANSFER_OFF;
        SWUART_TX_COMPLETE_CALL_ON;
    }
    return G_buffer_tx[G_n_tx];
}

void callback_transfer_complete(void)
{
    SW_RS485_IN;
    SWUART_RECEIVE_ON;
}
