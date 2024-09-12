// ssd1306.c

#include "Oled1306.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wim_ios.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


static const char *TAGI2C = "I2C";

#define I2C_BUS_PORT  0

#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0x40

//extern i2c_master_dev_handle_t dev_handle;
i2c_master_dev_handle_t dev_handle;

static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static void write_command(uint8_t command) {
    uint8_t data[2] = {SSD1306_COMMAND, command};
    i2c_master_transmit(dev_handle, data, 2, -1); // o -1 era 10
}

void ssd1306_init() {

    ESP_LOGI(TAGI2C, "Initialize I2C bus");

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = OLED_SDA,
        .scl_io_num = OLED_SCL,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
    };

    //i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));    

    // Initialize display
    write_command(0xAE); // Display off
    write_command(0xD5); // Set display clock divide ratio/oscillator frequency
    write_command(0x80); // Suggested ratio 0x80
    write_command(0xA8); // Set multiplex ratio (1 to 64)
    write_command(0x3F); // 1/64 duty
    write_command(0xD3); // Set display offset
    write_command(0x00); // No offset
    write_command(0x40); // Set start line address
    write_command(0x8D); // Set charge pump
    write_command(0x14); // Enable charge pump
    write_command(0x20); // Set memory mode
    write_command(0x00); // Horizontal addressing mode
    write_command(0xA1); // Set segment re-map
    write_command(0xC8); // Set COM output scan direction
    write_command(0xDA); // Set COM pins hardware configuration
    write_command(0x12);
    write_command(0x81); // Set contrast control
    write_command(0xCF);
    write_command(0xD9); // Set pre-charge period
    write_command(0xF1);
    write_command(0xDB); // Set VCOMH deselect level
    write_command(0x40);
    write_command(0xA4); // Enable entire display on
    write_command(0xA6); // Set normal display
    write_command(0xAF); // Display on

    ssd1306_clear();
    ssd1306_display();
}

void ssd1306_clear(void) {
    memset(buffer, 0, sizeof(buffer));
}

void ssd1306_display(void) {
    write_command(0x21); // Set column address
    write_command(0);    // Start address
    write_command(127);  // End address

    write_command(0x22); // Set page address
    write_command(0);    // Start address
    write_command(7);    // End address

    uint8_t data[SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1];
    data[0] = SSD1306_DATA;
    memcpy(data + 1, buffer, sizeof(buffer));
    i2c_master_transmit(dev_handle, data, sizeof(data), -1);
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    
    if (color) {
        buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}


void ssd1306_draw_char(uint8_t x, uint8_t y, char c, const Font *font, uint8_t color) {
    if (font->width > 8 || font->height > 16) {
        // This function supports fonts up to 8x16
        return;
    }

    uint8_t col, row;
    uint16_t char_offset = (c - 32) * ((font->width * font->height + 7) / 8);
    //uint16_t char_offset = (c - 32) * font->width * 2; // For 8x16, each character is 16 bytes (8 columns * 2 bytes per column)

    for (col = 0; col < font->width; col++) {
        uint8_t upper_byte = font->data[char_offset + col];
        uint8_t lower_byte = font->data[char_offset + font->width + col]; // Second byte is after all upper bytes

        for (row = 0; row < 8; row++) {
            if (upper_byte & (1 << (row))) {
                ssd1306_draw_pixel(x + col, y + row, color);
            }
            if (font->height > 8 && (lower_byte & (1 << (row)))) {
                ssd1306_draw_pixel(x + col, y + 8 + row, color);
            }
        }
    }
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str, const Font *font, uint8_t color) {
    if (font->width > 8 || font->height > 16) {
        // This function supports fonts up to 8x16
        return;
    }

    while (*str) {
        ssd1306_draw_char(x, y, *str++, font, color);
        x += font->width;
        if (x + font->width > SSD1306_WIDTH) {
            x = 0;
            y += font->height;
            if (y + font->height > SSD1306_HEIGHT) {
                break;
            }
        }
    }
}