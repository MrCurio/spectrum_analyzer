#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_dsp.h"
#include <math.h>
#include "driver/uart.h"

#include "fft_uart.h"


#define TAG "FFT_UART"

static QueueHandle_t data_queue;
static uint8_t fft_buffer[BUFFER_FFT];

static float y_cf[BUFFER_FFT * 2];
static float *y1_cf = &y_cf[0];
float wind[BUFFER_FFT];
double vReal[BUFFER_FFT];


void renderFFT(void *param);

void init_fft_thread(){

    data_queue = xQueueCreate(2, sizeof( uint8_t ) * BUFFER_FFT );

    // Hilo que se va a encargar de procesar los datos de Bluetooth
    xTaskCreatePinnedToCore(renderFFT,      // Function that should be called
                            "FFT Renderer", // Name of the task (for debugging)
                            2048,          // Stack size (bytes)
                            NULL,           // Parameter to pass
                            1,              // Task priority
                            NULL,           // Task handle
                            1               // Core you want to run the task on (0 or 1)
    );

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TX_FFT, RX_FFT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //Init FFT DSP
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, BUFFER_FFT));
    dsps_wind_hann_f32(wind, BUFFER_FFT); //Creamos una ventana tipo hanning

    
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Error number: %d", err);
    //     esp_restart();
    // }  
    
}


void process_stream(const uint8_t *data){ //(void *) &valor
    int byteOffset = 0;
    for (int i = 0; i < BUFFER_FFT; i++) {
      int16_t sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      int16_t sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      byteOffset = byteOffset + 4;
    }

    xQueueSend(data_queue, (void *)&vReal, ( TickType_t ) 0 ); //Non-blocking if queue is full
}

void renderFFT(void *param){

    while (1)
    {
        if( xQueueReceive( data_queue, fft_buffer, portMAX_DELAY)){

            for (int i = 0; i < BUFFER_FFT; i++)
            {
                y_cf[i * 2 + 0] = fft_buffer[i] * wind[i];
                y_cf[i * 2 + 1] = 0;
            }

            dsps_fft2r_fc32(y_cf, BUFFER_FFT);
            dsps_bit_rev_fc32(y_cf, BUFFER_FFT);
            dsps_cplx2reC_fc32(y_cf, BUFFER_FFT);

            for (int i = 0; i < BUFFER_FFT / 2; i++)
            {
                y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / BUFFER_FFT);
            }

            //dsps_view(y1_cf, BUFFER_FFT / 2, 128, 30, -20, 100, '|');
            
        }
    }
    
}