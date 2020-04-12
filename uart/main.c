/*
 * main.h
 *
 * Created: 2020.04.11 - 2020.04.12
 *  Author: GriDev
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "../macros.h"
#include "main.h"


//////// Variables

uint8_t G_buffer_tx[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t G_n_tx = 10;

uint8_t G_buffer_rx[10];
uint8_t G_n_rx = 0;


//////// Private Functions

//// Callback functions from UART ISR

void receive_byte(uint8_t ucsra, uint8_t frame_data);
uint8_t transmit_byte(void);
void callback_transfer_complete(void);


////////

int main(void)
{
    sei();
    uart_init(receive_byte, transmit_byte);
    uart_set_baud(UART_SPEED);
    set_callback_uart_tx_complete(callback_transfer_complete);

    UART_RECEIVE_OFF;
    RS485_OUT;
    UART_TRANSFER_ON;

    for ( ; ; ) {
        volatile uint8_t* n_rx_p = &G_n_rx;
            // Нам нужен именно указатель на volatile переменную,
            // чтобы компилятор корректно применил оптимизацию.
        if (*n_rx_p >= 3) {
            if (G_buffer_rx[0] == 0 &&
                G_buffer_rx[1] == 1 &&
                G_buffer_rx[2] == 2 ) {

                UART_RECEIVE_OFF;
                G_n_rx = 0;

                G_n_tx = 3;
                G_buffer_tx[7] = 4;
                G_buffer_tx[8] = 5;
                G_buffer_tx[9] = 6;
                
                RS485_OUT;
                UART_TRANSFER_ON;
            }
        }
        _NOP;
    }

}

void receive_byte(uint8_t ucsra, uint8_t frame_data)
{
    if (G_n_rx < sizeof G_buffer_rx) {
        G_buffer_rx[G_n_rx] = frame_data;
        G_n_rx++;
        if (UART_RECEIVE_FRAME_ERROR(ucsra)) {
            // Ошибка приёма кадра
        }
        if (UART_RECEIVE_OVERFLOW(ucsra)) {
            // Ошибка приёма кадра
        }
    } else {
        // Переполнение буфера
    }
}

uint8_t transmit_byte(void)
{
    G_n_tx--;
    if (G_n_tx == 0) {
        UART_TRANSFER_I_OFF;
        UART_TX_COMPLETE_CALL_ON;
    }
    return G_buffer_tx[sizeof G_buffer_tx - 1 - G_n_tx];
}

void callback_transfer_complete(void)
{
    RS485_IN;
    UART_TRANSFER_OFF;
    UART_RECEIVE_ON;
}
