#ifndef __MCP3564_H__
#define __MCP3564_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "driver/gpio.h"

// Custom macros
#define N_CHANNELS      6
#define N_SAMPLES       16384
#define N_SAMPLES_MASK  N_SAMPLES-1

typedef struct{
    // Pin mapping
    const gpio_num_t gpio_num_miso;
    const gpio_num_t gpio_num_mosi;
    const gpio_num_t gpio_num_clk;
    const gpio_num_t gpio_num_cs;
    const gpio_num_t gpio_num_pwm;
    const gpio_num_t gpio_num_ndrdy;

    const int spi_clk_hz;

    // External flag 
    uint32_t flag_drdy;
} MCP3564_t;

void MCP3564_startUp(MCP3564_t* mcp_obj);
uint32_t MCP3564_readVoltage(MCP3564_t* mcp_obj);
void MCP3564_pause(void);
void MCP3564_resume(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif