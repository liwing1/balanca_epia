
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
#include "Perifericos.h"
#include "driver/gptimer.h" 
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#include "Loop.h"


int32_t Media_loop=17200;
int64_t Media_64=17200<<8;

bool Tem_veiculo=false;
#define ALFA 0.01


#define PCNT_HIGH_LIMIT       4000
#define PCNT_INPUT_GPIO       LOOP1   // Change this to the GPIO you're using for PCNT input

#define QUEUE_LENGTH          1024
#define QUEUE_ITEM_SIZE       sizeof(uint32_t)
static const char *TAG_PCNT = "PCNT";

// PCNT unit handle
pcnt_unit_handle_t pcnt_unit = NULL;

// ESP Timer handle
esp_timer_handle_t periodic_timer;

// Time variables
uint64_t start_time = 0;

// Queue implementation
static uint32_t queue[QUEUE_LENGTH];
static int pe = 0, ps = 0;


uint32_t dequeue() {
    if (ps == pe) return queue[ps]; // Queue is empty
    ps = (ps + 1) & (QUEUE_LENGTH - 1);
    return queue[ps];
}

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {

    uint64_t end_time = esp_timer_get_time();

    //gpio_set_level(LED_YELLOW,1);

    uint64_t dt=end_time - start_time;

    // Poe na Fila
    pe = (pe + 1) & (QUEUE_LENGTH - 1);
    queue[pe] = (uint32_t)dt;

    pcnt_unit_stop(pcnt_unit);


    //se tiver veiculo nao atualiza a media
    //if (Tem_veiculo==false){
        Media_64=Media_64-(Media_64>>8)+dt;
        Media_loop=(int) (Media_64>>8);
    //}

    //Se o valor do loop for menor que media - 50, tem veiculo
    if (dt < Media_loop-30){
        Tem_veiculo=true;
    }    

    //Se o valor do loop for maior que media - 30, nao tem veiculo
    else if (dt > Media_loop-15){
        Tem_veiculo=false;
    }


    //gpio_set_level(LED_YELLOW,0);    
    return true;
}



void Init_loop(){

    // Initialize PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 100,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_INPUT_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    //ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));


    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // add high limit watch point
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 3600));

    // Enable PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));

    ESP_LOGI(TAG_PCNT, "Starting timer and PCNT...");

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit)); 



}




