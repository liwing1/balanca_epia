#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "esp_async_memcpy.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MCP3564.h"

#define BUTTON_PIN  38

MCP3564_t MCP_instance = {
    .gpio_num_pwm       = GPIO_NUM_9,
    .gpio_num_ndrdy     = GPIO_NUM_10,
    .gpio_num_miso      = GPIO_NUM_11,
    .gpio_num_cs        = GPIO_NUM_12,
    .gpio_num_clk       = GPIO_NUM_13,
    .gpio_num_mosi      = GPIO_NUM_14,
    .flag_drdy = 0,
};


void monitor_task(void* p)
{
    while(1)
    {   
        for(uint8_t i = 0; i < 6; i++) {
            printf("%04.2f\t", MCP_instance.history[0][i] * (3.3/8388608));
        }
        printf("%ld\n", freq);

        freq = 0;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    MCP3564_startUp(&MCP_instance);

    // First enable interrupts
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_cfg);

    printf("Start Conv\n");
    // MCP3564_startConversion(&MCP_instance);

    xTaskCreatePinnedToCore(monitor_task, "monitor", 2048*4, NULL, 5, NULL, APP_CPU_NUM);
}