#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_avrc_api.h"

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

bool new_song = false;
uint8_t song_title[30] = {0};
uint8_t song_artist_len = 0;
uint8_t song_artist[20] = {0};

void add_new_song_info(uint8_t attr_id, uint8_t *text, int len){
    switch (attr_id)
    {
    case ESP_AVRC_MD_ATTR_TITLE:
        memset(song_title, 0, sizeof(song_title));
        memcpy(song_title + 1, text, len);
        break;
    case ESP_AVRC_MD_ATTR_ARTIST:
        memset(song_artist, 0, sizeof(song_artist));
        song_artist_len = 0;
        memcpy(song_artist, text, len);
        song_artist_len = song_artist_len + len;
        new_song = true;
        break;
    default:
        break;
    } 
}

void renderFFT(void *param);

void init_fft_thread(){

    data_queue = xQueueCreate(2, sizeof( uint8_t ) * BUFFER_FFT );

    // Hilo que se va a encargar de procesar los datos de Bluetooth
    xTaskCreatePinnedToCore(renderFFT,      // Function that should be called
                            "FFT Renderer", // Name of the task (for debugging)
                            2048,          // Stack size (bytes)
                            NULL,           // Parameter to pass
                            10,              // Task priority
                            NULL,           // Task handle
                            1               // Core you want to run the task on (0 or 1)
    );

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TX_FFT, RX_FFT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

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

            for (int i = 0; i < N_SAMPLES / 2; i++)
            {
                y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / BUFFER_FFT);
                // if(i % 10 == 0){
                //     ESP_LOGI(TAG, "Valor %d: %f",i, y1_cf[i]);
                // }
                //vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            //dsps_view(y1_cf, N_SAMPLES / 2, 14, 20, -60, 40, '|');       


            /* Algoritmo para procesar los datos por bandas */
            // 1. Determinamos los máximos y mínimos globales y escalamos con respecto a ellos
            uint8_t *data_uart = malloc(sizeof(int) * 12);
            uint8_t *data_final = malloc(sizeof(uint8_t) * 12);
            
            int offset = 0;
            int suma = 0;
            int crc = 0;
            for (int i = 0; i < N_SAMPLES/2; i++)
            {
                if (i <= 3) //0-65 Hz
                {
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 3)
                    {
                        suma = suma/4;   
                        crc = crc + suma;
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }                                   
                }
                else if(i <= 5){ //65-100 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 5)
                    {
                        suma = suma/2;    
                        crc = crc + suma;
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 9){ //100-200 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 9)
                    {
                        suma = suma/4;    
                        crc = crc + suma;
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 14){ //200-300 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 14)
                    {
                        suma = suma/6;   
                        crc = crc + suma; 
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 23){ //300-500 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 23)
                    {
                        suma = suma/10;   
                        crc = crc + suma; 
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 37){ //500-800 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 37)
                    {
                        suma = suma/15;  
                        crc = crc + suma;  
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 47){ //800-1k Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 47)
                    {
                        suma = suma/11; 
                        crc = crc + suma;   
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 70){ //1k-1500 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 70)
                    {
                        suma = suma/24; 
                        crc = crc + suma;   
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 140){ //1500-3000 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 140)
                    {
                        suma = suma/71;    
                        crc = crc + suma;
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 372){ //3000-8000 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 372)
                    {
                        suma = suma/233;  
                        crc = crc + suma;  
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else if(i <= 560){ //8000-12000 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 560)
                    {
                        suma = suma/189;   
                        crc = crc + suma; 
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }
                else { //12000-22000 Hz
                    suma = suma + (int)(y1_cf[i] + 150);
                    if (i == 1023)
                    {
                        suma = suma/464; 
                        crc = crc + suma;   
                        memcpy(data_uart + offset, &suma, sizeof(suma));
                        offset = offset + sizeof(suma); 
                        suma = 0;
                    }   
                }          
                
            }

            //memcpy(data_uart + offset, &crc, sizeof(crc));


            // int valores[13] = {0};
            // memcpy(valores, data_uart, sizeof(int) * 13);

            // for (int i = 0; i < 13; i++)
            // {
            //     ESP_LOGI(TAG, "Valores UART: %d:  %d", i, valores[i]);
            // }

            for (int i = 0; i < 12; i++)
            {
                data_final[i] = data_uart[i*4];
            }
            
            //uint8_t rcv[3] = {0};
            // int t_len = uart_write_bytes(UART_NUM_1, (const char *) rcv, 3);
            // ESP_LOGI(TAG, "Len valores enviados: %d", t_len);
            // if (t_len <= 0)
            // {
            //     ESP_LOGE(TAG, "ERROR SENDING TO UART -> TX");
            // }

            // vTaskDelay(10 / portTICK_PERIOD_MS);
            // uint8_t rcv[52] = {0};
            // for (int i = 0; i < 52; i++)
            // {
            //     rcv[i] = i;
            // }
            
            //ESP_LOGI("Test", "Valores son: %d:%d:%d", rcv[0], rcv[1], rcv[2]);
            //int len = uart_write_bytes(UART_NUM_1, (const char *) data_uart, sizeof(int) * 3);
            //ESP_LOGI(TAG, "Valores enviados: %d", len);
            if (new_song)
            { 
                uint8_t test[12] = "}Nicky Jam{";
                song_title[0] = '}';
                song_artist[song_artist_len] = '{';
                uint8_t buffer_info[48] = {0};
                memcpy(buffer_info, song_title, 28);
                memcpy(buffer_info + 28, song_artist, 20);
                ESP_LOGI(TAG, "Titulo: %s   Artista: %s", song_title, song_artist);
                uart_write_bytes(UART_NUM_1, (const char *) buffer_info, 48); 
                ESP_LOGI(TAG, "Valor guardado: %s", buffer_info);
                //uart_write_bytes(UART_NUM_1, (const char *) test, 12); 
                // uart_write_bytes(UART_NUM_1, (const char *) song_artist, 20); 
                new_song = false;
            }
            else {
                uart_write_bytes(UART_NUM_1, (const char *) data_final, sizeof(uint8_t) * 12);
            }
            
            // uint8_t *data_rcv = malloc(sizeof(int) * 13);
            // int len = uart_read_bytes(UART_NUM_1, data_rcv, sizeof(int) * 13, portMAX_DELAY);
            // ESP_LOG_BUFFER_HEX(TAG, data_rcv, sizeof(int) * 13);
            // free(data_rcv);
            // for (int i = 0; i < 12; i++)
            // {
            //     ESP_LOGI("TAG", "Valor: %d      %d", i, data_final[i]);
            // }
            
            // ESP_LOGI("TAG", "------------------------------------------");


            free(data_uart);
            free(data_final);
            

            vTaskDelay(100 / portTICK_PERIOD_MS);
            
        }
    }
    
}