
#ifndef USR_IOS_H
#define USR_IOS_H

#include "driver/gpio.h"

#define LED_RED_IO          GPIO_NUM_37
#define LED_YELLOW_IO       GPIO_NUM_36
#define LED_GREEN_IO        GPIO_NUM_35

#define BOTAO_A_IO          GPIO_NUM_38
#define BOTAO_B_IO          GPIO_NUM_39
#define BOTAO_C_IO          GPIO_NUM_40

#define OLED_SDA            GPIO_NUM_34
#define OLED_SCL            GPIO_NUM_48

typedef enum{
    BOTAO_A,
    BOTAO_B,
    BOTAO_C
} buttons_t;

typedef enum{
    LED_GREEN,
    LED_YELLOW,
    LED_RED
} leds_t;

typedef enum{
    LED_ON,
    LED_OFF
} leds_state_t;

void usr_ios_Init(void);
bool usr_io_is_button_pressed(buttons_t button_idx);
void usr_io_set_led(leds_t led_color, leds_state_t led_state);

#endif // USR_IOS_H
