/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_lcd_panel_vendor.h"

#include "MCP3564.h"

static const char *TAG = "example";

MCP3564_t MCP_instance = {
    .gpio_num_pwm       = GPIO_NUM_9,
    .gpio_num_ndrdy     = GPIO_NUM_10,
    .gpio_num_miso      = GPIO_NUM_11,
    .gpio_num_cs        = GPIO_NUM_12,
    .gpio_num_clk       = GPIO_NUM_13,
    .gpio_num_mosi      = GPIO_NUM_14,
    .flag_drdy = 0,
};

#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           34
#define EXAMPLE_PIN_NUM_SCL           48
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

extern void example_lvgl_demo_ui_start(lv_disp_t *disp);
extern void example_lvgl_ui_create_timer(double* f_loop1, double* f_loop2);
extern void pulse_counter_start(void);
extern double pulse_counter_get_freq_a(void);
extern double pulse_counter_get_freq_b(void);

void monitor_task(void* p)
{
    MCP3564_pause();
    MCP_instance.flag_drdy = 0;

    while(1)
    {   
        if(gpio_get_level(GPIO_NUM_38) == 0)
        {
            MCP3564_resume();
            while(MCP_instance.flag_drdy < N_SAMPLES_MASK) {
                printf("%ld\n", MCP_instance.flag_drdy);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            MCP3564_pause();

            printf("ch4,ch5,f1\n");

            for(uint32_t i = 0; i < N_SAMPLES; i++) 
            {
                printf("%06.4f,", history[i][4] * (3.3/8388608));
                printf("%06.4f,", history[i][5] * (3.3/8388608));
                printf("%lf\n", history_f[i]);

                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    double loop1 = 0;
    // double loop2 = 0;
    double loop1_max = 0;

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    if (lvgl_port_lock(0)) {
        example_lvgl_demo_ui_start(disp);
        example_lvgl_ui_create_timer(&loop1, &loop1_max);

        lvgl_port_unlock();
    }

    pulse_counter_start();
    MCP3564_startUp(&MCP_instance);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_38,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&btn_cfg);

    xTaskCreatePinnedToCore(monitor_task, "monitor_task", 2048*4, NULL, tskIDLE_PRIORITY + 5, NULL, PRO_CPU_NUM);

    while(1) 
    {
        for(uint32_t i = 0; i < N_SAMPLES; i++)
        {
            loop1 = pulse_counter_get_freq_a();
            // loop2 = pulse_counter_get_freq_b();

            if(loop1 > loop1_max) {
                loop1_max = loop1;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}