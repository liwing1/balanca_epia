/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "wim_ios.h"
#include "Oled1306.h"
#include "fonts.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
//#include "..\components\MCP3564\MCP3564.h"
#include "MCP3564.h"
#include "Perifericos.h"
#include "driver/gptimer.h" 
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "Loop.h"

#define TIMER_INTERVAL_MS     (20)  // 20 ms timer interval

extern int32_t Media_loop;
extern bool Tem_veiculo;

static const char *TAG = "MAIN";

static uint32_t Delta_T=0;

extern uint64_t start_time;
extern pcnt_unit_handle_t pcnt_unit;
extern esp_timer_handle_t periodic_timer;
extern bool flag_captura_adc;
extern int32_t pe_adc;

static void IRAM_ATTR timer_callback(void* arg) {
    start_time = esp_timer_get_time();
    //gpio_set_level(LED_RED,1);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
    debounce_botoes();    
    //gpio_set_level(LED_RED,0); 

}

// task to update de oled
void task_oled(void *pvParameter) {
    while (1) {
        // A atualizacao do Oled demora cerca de 20 ms
        char str[16];

        ssd1306_clear();
        ssd1306_draw_string(0,  0, "Panavideo", &Font_8x16,1);


        sprintf(str, "DT = %lu", Delta_T);
        ssd1306_draw_string(0, 32, str, &Font_8x16,1); 

        // Mostra a media de loop
        sprintf(str, "Media = %d", (int) Media_loop);
        ssd1306_draw_string(0, 48, str, &Font_8x16,1);

        ssd1306_display();
      
        vTaskDelay(80 / portTICK_PERIOD_MS); // Small delay to prevent watchdog trigger
    }
}

//#define ARRAY_SIZE (256 * 1024)  // 1024 KB (1 MB) in 32-bit words

//uint32_t *queue_adc;


void app_main(void) {
    ESP_LOGI(TAG, "Initializing...");

    // queue_adc = (uint32_t *)heap_caps_malloc(ARRAY_SIZE * sizeof(uint32_t), MALLOC_CAP_SPIRAM);

    // // Check if the allocation was successful
    // if (queue_adc == NULL) {
    //     ESP_LOGE("PSRAM", "Failed to allocate memory in PSRAM");
    //     return;
    // } else {
    //     ESP_LOGI("PSRAM", "Successfully allocated %d bytes in PSRAM", ARRAY_SIZE * sizeof(uint32_t));
    // }

    // Escreva algo no display
    ssd1306_init();
    ssd1306_clear();
    ssd1306_draw_string(0,  0, "Panavideo", &Font_8x16,1);
    ssd1306_draw_string(10, 32, "Iniciando", &Font_5x7,1);
    ssd1306_display();

    // Configure botoes pins as inputs
    Botoes_init();

    // Configure LED pins as outputs
    Leds_init();

    // Initialize UART
    Uart_init();

    // Inicializa Loop
    Init_loop();

    // Initialize Timer
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &timer_callback,
        .name = "periodic"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));   

    // Start the timer
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_INTERVAL_MS * 1000));

    //create a task to update the oled
    xTaskCreate(&task_oled, "task_oled", 8192, NULL, 5, NULL);


    // Inicializa o ADC com a SPI e o gerador de clock
    MCP3564_startUp();

    int32_t pe1=-1;

    while (1) {

        uint32_t delta_time = dequeue();

        if (delta_time > 0) {
            //printf("%lu\n", delta_time);
            Delta_T= delta_time;
        }

        // se o Botao_A for pressionado, para acaptura do acd e imprime
        if (Botao_A()){
            gpio_set_level(LED_RED,0);
            flag_captura_adc=false;
            vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to stop

            //imprime 32 amostras do buffer do ADC
            for (int i=0;i<32;i++){

                //tira da fila
                int32_t dado = dequeue_adc();

                // canal nos bits 32,31,30,29
                int canal = 5-((dado>>28)&0x0F);

                // valor nos bits 27 a 0
                int32_t valor = dado&0xFFFFFF;

                // imprima o canal e o valor como hexadecimal
                printf("dado = %lX\n",dado);               
                printf("Canal %X: %f\n",canal,(float)valor*3.3/8388606.0);

            }
            vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to continue
            flag_captura_adc=true;
            while (Botao_A()) {
                vTaskDelay(1); // Small delay to prevent watchdog trigger
            };
            gpio_set_level(LED_RED,1);
        }

    // se Tem_veiculo for true e pe1=0, entao anota o pe_adc

        if (Tem_veiculo && (pe1==-1)){
            pe1=pe_adc;
            //printf(">pe1 = %ld\n",pe1);
        }

        // terminou o veiculo => imprime
        if ((!Tem_veiculo) && (pe1!=-1)){
            //printf(">pe_adc = %ld\n",pe_adc);

            int32_t valor[2];

            vTaskDelay(190/portTICK_PERIOD_MS); // tempo para acabar veiculo
            flag_captura_adc=false;
            vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to stop

            int32_t amostras=(pe_adc-pe1+1500)&(QUEUE_ADC_LENGTH-1);
            //imprime todo o buffer do ADC
            // printf("Amostras = %ld\n",amostras);
            // printf("pe1 = %ld\n",pe1);
            // printf("pe_adc = %ld\n",pe_adc);
            for (int i=0;i<amostras;i++){

                //tira da fila
                int32_t dado = dequeue_adc();

                // canal nos bits 32,31,30,29
                int canal = 5-((dado>>28)&0x0F);

                // valor nos bits 27 a 0
                valor[canal] = dado&0xFFFFFF;

                //tira da fila
                dado = dequeue_adc();
                int32_t dado1 = dado;

                // canal nos bits 32,31,30,29
                canal = 5-((dado>>28)&0x0F);

                // valor nos bits 27 a 0
                valor[canal] = dado&0xFFFFFF;              

                // imprima o canal e o valor como hexadecimal
                printf("%lX, %lX\n",dado,dado1);
                printf("%f, %f\n",(float)valor[0]*3.3/8388606.0,(float)valor[1]*3.3/8388606.0);
                //if((i&31)==0) vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to stop
                vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to stop

            }

            pe1=-1;
            vTaskDelay(10/portTICK_PERIOD_MS); // Small delay to allow real time to continue
            flag_captura_adc=true;
        }

        // atualiza o led verde com o estado de tem veiculo
        gpio_set_level(LED_GREEN,!Tem_veiculo);

        // Copia botoes para Led - teste
        //int s;
        //s=Botao_A();
        //gpio_set_level(LED_RED,s);
        //s=Botao_B();
        //gpio_set_level(LED_YELLOW,s);
        //s=Botao_C();
        //gpio_set_level(LED_GREEN,s);

        vTaskDelay(1); // Small delay to prevent watchdog trigger        
    }

}    