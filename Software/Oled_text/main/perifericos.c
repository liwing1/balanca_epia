#include "wim_ios.h"
#include "driver/gpio.h"
#include "driver/uart.h"

int BOTAO_A_STATE = 1;
int BOTAO_B_STATE = 1;
int BOTAO_C_STATE = 1;

static int button_state[3] = {1, 1, 1};
static int button_counter[3] = {0, 0, 0};

void Leds_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_RED) | (1ULL << LED_YELLOW) | (1ULL << LED_GREEN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // coloque os led em off
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_YELLOW, 1);
    gpio_set_level(LED_GREEN, 1);
}    

void Uart_init() {
    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

void Botoes_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BOTAO_A_IO) | (1ULL << BOTAO_B_IO) | (1ULL << BOTAO_C_IO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

void debounce_botoes() {
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
}

int Botao_A() {
    return (button_state[0]^1);
}

int Botao_B() {
    return (button_state[1]^1);
}
int Botao_C() {
    return (button_state[2]^1);
}