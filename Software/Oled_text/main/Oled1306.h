// ssd1306.h

#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
//#include "driver/i2c_master.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

typedef struct {
    uint8_t width;
    uint8_t height;
    const uint8_t *data;
} Font;

void ssd1306_init();
void ssd1306_clear(void);
void ssd1306_display(void);
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, const Font *font, uint8_t color);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str, const Font *font, uint8_t color);

#endif // SSD1306_H