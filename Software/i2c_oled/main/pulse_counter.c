/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "esp_timer.h"

static const char *TAG = "pcnt";

#define EXAMPLE_PCNT_HIGH_LIMIT SHRT_MAX/2
#define EXAMPLE_PCNT_LOW_LIMIT  (-EXAMPLE_PCNT_HIGH_LIMIT - 1)

#define EXAMPLE_EC11_GPIO_A 10
#define EXAMPLE_EC11_GPIO_B 11

static QueueHandle_t queue_a;
static QueueHandle_t queue_b;
static double freq_khz_a = 0;
static double freq_khz_b = 0;

void timer_callback(void* arg) {
    // Your code here
    printf("Timer triggered\n");
}

void pulse_counter_handler_a(void* p)
{
    // Report counter value
    int64_t event_count = 0;

    printf("Freq (Hz)\n");
    while (1) {
        if (xQueueReceive(queue_a, &event_count, pdMS_TO_TICKS(1000))) {
            freq_khz_a = (double)(((double)EXAMPLE_PCNT_HIGH_LIMIT/(double)event_count)*1e3);
            // ESP_LOGI(TAG, "Watch point event, f = %lf Hz", f*10e6);

            printf("%lf, %lf\n", (double)freq_khz_a, (double)freq_khz_b);
        } else {
            ESP_LOGI(TAG, "Timeout pulse count A");
        }
    }
}

void pulse_counter_handler_b(void* p)
{
    // Report counter value
    int64_t event_count = 0;

    printf("Freq (Hz)\n");
    while (1) {
        if (xQueueReceive(queue_b, &event_count, pdMS_TO_TICKS(1000))) {
            freq_khz_b = (double)(((double)EXAMPLE_PCNT_HIGH_LIMIT/(double)event_count)*1e3);
            // ESP_LOGI(TAG, "Watch point event, f = %lf Hz", f*10e6);

            // printf("%lf\n", (double)freq_khz_b);
        } else {
            ESP_LOGI(TAG, "Timeout pulse count B");
        }
    }
}

static bool example_pcnt_on_reach_a(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    static int64_t offset_timer_us = 0;
    static int64_t current_timer_us = 0;

    current_timer_us = esp_timer_get_time();
    int64_t dt = (current_timer_us - offset_timer_us);
    offset_timer_us = current_timer_us;
    
    xQueueSendFromISR(queue, &dt, &high_task_wakeup);
    // xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

static bool example_pcnt_on_reach_b(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    static int64_t offset_timer_us = 0;
    static int64_t current_timer_us = 0;

    current_timer_us = esp_timer_get_time();
    int64_t dt = (current_timer_us - offset_timer_us);
    offset_timer_us = current_timer_us;
    
    xQueueSendFromISR(queue, &dt, &high_task_wakeup);
    // xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void pulse_counter_start(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit_a = NULL;
    pcnt_unit_handle_t pcnt_unit_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_b));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_a, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_b, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = -1,
    };
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_a, &chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_b, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_a, EXAMPLE_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_b, EXAMPLE_PCNT_HIGH_LIMIT));

    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach_a,
    };
    queue_a = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_a, &cbs, queue_a));
    
    cbs.on_reach = example_pcnt_on_reach_b;
    queue_b = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_b, &cbs, queue_b));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_b));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_b));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_a));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_b));

    // Create a timer configuration structure
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,  // Pointer to the callback function
        .arg = NULL,                  // Optional argument passed to the callback
        .dispatch_method = ESP_TIMER_TASK,  // Method for dispatching the callback
        .name = "my_timer"            // Timer name
    };
    // Create the timer
    esp_timer_handle_t timer_handle;
    esp_err_t err = esp_timer_create(&timer_args, &timer_handle);
    if (err != ESP_OK) {
        printf("Failed to create timer: %s\n", esp_err_to_name(err));
        return;
    }

    xTaskCreatePinnedToCore(
        pulse_counter_handler_a,
        "pulse_counter_handler_a", 
        2048*2, 
        NULL, 
        configMAX_PRIORITIES-2, 
        NULL,
        PRO_CPU_NUM
    );

    xTaskCreatePinnedToCore(
        pulse_counter_handler_b,
        "pulse_counter_handler_b", 
        2048*2, 
        NULL, 
        configMAX_PRIORITIES-2, 
        NULL,
        APP_CPU_NUM
    );
}

double pulse_counter_get_freq_a(void)
{
    return freq_khz_a;
}

double pulse_counter_get_freq_b(void)
{
    return freq_khz_b;
}