#ifndef __MCP3564_H__
#define __MCP3564_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "driver/gpio.h"

#define MCP3564_BUFFER_SIZE 4096
#define MCP3564_BUFFER_MASK MCP3564_BUFFER_SIZE-1

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
    uint32_t buffer[MCP3564_BUFFER_SIZE];
} MCP3564_t;

void MCP3564_startUp(MCP3564_t* mcp_obj);
void MCP3564_startConversion(MCP3564_t* mcp_obj);
uint32_t MCP3564_readVoltage(MCP3564_t* mcp_obj);
uint32_t MCP3564_readChannels(MCP3564_t* mcp_obj, float* buffer);
int32_t MCP3564_signExtend(uint32_t Bytes);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif