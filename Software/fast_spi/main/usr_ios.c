#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usr_ios.h"

static int button_state[3] = {1, 1, 1};
static int button_counter[3] = {0, 0, 0};

static void debounce_botao(void* p) {
    while(1) {
        for (int i = 0; i < 3; i++) {
            gpio_num_t button = BOTAO_A_IO + i;
            int current_state = gpio_get_level(button);
            if (current_state != button_state[i]) {
                button_counter[i]++;
                if (button_counter[i] >= 3) { // 3 * 20ms = 60ms debounce time
                    button_state[i] = current_state;
                    button_counter[i] = 0;
                }
            } else {
                button_counter[i] = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void usr_ios_Init(void) {
    // Init Buttons
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BOTAO_A_IO) | (1ULL << BOTAO_B_IO) | (1ULL << BOTAO_C_IO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Init LEDs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_RED_IO) | (1ULL << LED_YELLOW_IO) | (1ULL << LED_GREEN_IO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // coloque os led em off
    gpio_set_level(LED_RED_IO, 1);
    gpio_set_level(LED_YELLOW_IO, 1);
    gpio_set_level(LED_GREEN_IO, 1);

    xTaskCreate(debounce_botao, "debounce_botao", 2048*3, NULL, tskIDLE_PRIORITY+5, NULL);
}

bool usr_io_is_button_pressed(buttons_t button_idx) {
    return (button_state[button_idx] == 0);
}

void usr_io_set_led(leds_t led_color, leds_state_t led_state) {
    gpio_set_level(LED_GREEN_IO + led_color, led_state);
}