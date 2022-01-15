idf_component_register(SRCS "bt_app_core.c" 
                            "bt_app_av.c" 
                            "bt_speaker.c" 
                            "fft_uart.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt audio_renderer esp-dsp)