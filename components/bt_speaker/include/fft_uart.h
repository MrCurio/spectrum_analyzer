#ifndef __FFT_UART_H__
#define __FFT_UART_H__

#include "driver/gpio.h"

#define BUFFER_FFT 4096
#define N_SAMPLES 2048

#define TX_FFT GPIO_NUM_16
#define RX_FFT GPIO_NUM_17

void init_fft_thread();

void process_stream(const uint8_t *data);
#endif