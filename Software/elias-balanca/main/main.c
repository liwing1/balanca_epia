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

// Create a semaphore used to report the completion of async memcpy
SemaphoreHandle_t semphr;
async_memcpy_config_t config = ASYNC_MEMCPY_DEFAULT_CONFIG();
async_memcpy_handle_t driver = NULL;
uint32_t buffer[MCP3564_BUFFER_SIZE] = {0};

// Callback implementation, running in ISR context
static bool my_async_memcpy_cb(async_memcpy_handle_t mcp_hdl, async_memcpy_event_t *event, void *cb_args)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)cb_args;
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(semphr, &high_task_wakeup); // high_task_wakeup set to pdTRUE if some high priority task unblocked
    return high_task_wakeup == pdTRUE;
}

MCP3564_t MCP_instance = {
    .gpio_num_pwm       = GPIO_NUM_9,
    .gpio_num_ndrdy     = GPIO_NUM_10,
    .gpio_num_miso      = GPIO_NUM_11,
    .gpio_num_cs        = GPIO_NUM_12,
    .gpio_num_clk       = GPIO_NUM_13,
    .gpio_num_mosi      = GPIO_NUM_14,
    .flag_drdy = 0,
    .buffer = {0}
};

void monitor_task(void* p)
{
    float volts[8] = {0};

    while(1)
    {
        if(gpio_get_level(BUTTON_PIN) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));

            MCP3564_readChannels(&MCP_instance, volts);

            printf("VOLTS\n");
            for (uint8_t i = 0; i < 8; i++) {
                printf("%d = %f\n", i, volts[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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

    semphr = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(esp_async_memcpy_install(&config, &driver)); // install driver with default DMA engine

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Start Conv\n");
    MCP3564_startConversion(&MCP_instance);

    xTaskCreatePinnedToCore(monitor_task, "monitor", 2048, NULL, 5, NULL, APP_CPU_NUM);
}