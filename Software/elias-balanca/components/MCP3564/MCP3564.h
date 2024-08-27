#ifndef __MCP3564_H__
#define __MCP3564_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "driver/gpio.h"

// Custom macros
#define SPI_CLOCK_SPEED 20000000 // 20 MHz

typedef struct{
    // Pin mapping
    const gpio_num_t gpio_num_miso;
    const gpio_num_t gpio_num_mosi;
    const gpio_num_t gpio_num_clk;
    const gpio_num_t gpio_num_cs;
    const gpio_num_t gpio_num_pwm;
    const gpio_num_t gpio_num_ndrdy;

    // External flag 
    uint32_t flag_drdy;

    // history
    uint32_t buffer[8];
} MCP3564_t;

void MCP3564_startUp(MCP3564_t* mcp_obj);
int32_t MCP3564_signExtend(uint32_t Bytes);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif