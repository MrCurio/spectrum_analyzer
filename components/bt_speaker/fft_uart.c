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

float x[N_SAMPLES];
static float y_cf[N_SAMPLES * 2];
static float *y1_cf = &y_cf[0];
float wind[N_SAMPLES];


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

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TX_FFT, RX_FFT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //Init FFT DSP
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, BUFFER_FFT));
    dsps_wind_hann_f32(wind, N_SAMPLES); //Creamos una ventana tipo hanning

    
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Error number: %d", err);
    //     esp_restart();
    // }  
    
}


void process_stream(const uint8_t *data){ //(void *) &valor

    xQueueSend(data_queue, (void *)data, ( TickType_t ) 0 ); //Non-blocking if queue is full
}

void renderFFT(void *param){

    while (1)
    {
        if( xQueueReceive( data_queue, fft_buffer, portMAX_DELAY)){

            int t = 0;
            static int16_t sample_l_int = 0;
            static int16_t sample_r_int = 0;
            static float sample_l_float = 0.0f;
            static float sample_r_float = 0.0f;
            static float in = 0.0f;

            for (uint32_t i = 0; i < N_SAMPLES; i += 4)
            {
                sample_l_int = (int16_t)((*(fft_buffer + i + 1) << 8) | *(fft_buffer + i));
                sample_r_int = (int16_t)((*(fft_buffer + i + 3) << 8) | *(fft_buffer + i + 2));
                sample_l_float = (float)sample_l_int / 0x8000;
                sample_r_float = (float)sample_r_int / 0x8000;
                in = (sample_l_float + sample_r_float) / 2.0f;
                x[t] = in * wind[t];
                t++;
            }           

            for (int i = 0; i < N_SAMPLES; i++)
            {
                y_cf[i * 2 + 0] = x[i];
                y_cf[i * 2 + 1] = 0;
            }

            dsps_fft2r_fc32(y_cf, N_SAMPLES);
            dsps_bit_rev_fc32(y_cf, N_SAMPLES);

            // for (int i = 0; i < N_SAMPLES / 2; i++)
            // {
            //     y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / BUFFER_FFT);
            // }

            // dsps_view(y1_cf, N_SAMPLES / 2, 128, 20, -60, 40, '|');
       
            // int t_len = uart_write_bytes(UART_NUM_2, (const char *) data, 9); //Send Request of 9 bytes = 8bytes + CRC
            // if (t_len <= 0)
            // {
            //     ESP_LOGE(TAG, "ERROR SENDING TO UART -> TX");
            // }

            float max = 0;
            float min = 0;
            int max_pos = 0;
            int min_pos = 0;
            for (int i = 0; i < N_SAMPLES/2; i++)
            {
                if (y1_cf[i] > max)
                {
                    max = y1_cf[i];
                    max_pos = i;
                }
                if (y1_cf[i] < min)
                {
                    min = y1_cf[i];
                    min_pos = i;
                }
            }          

            ESP_LOGI(TAG, "Max: %f      Min: %f     Max Pos: %d     Min Pos: %d", max, min, max_pos, min_pos);

            /* Algoritmo para procesar los datos por bandas */
            // 1. Determinamos los máximos y mínimos globales y escalamos con respecto a ellos

            // 2. Determinamos las bandas horizontales y verticales
            // Verticales: (max - min)/N_bandas_verticales
            // Horizontales: (N_Samples/2)/N_bandas_horizontales

            // 3. Clasificamos en bandas horizontales y verticales según posición del buffer y valor


            //4. Transmitimos por puerto serie el valor de dichas bandas del 1-N_bandas

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            
        }
    }
    
}