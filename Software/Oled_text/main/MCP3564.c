#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/_types.h>
#include "sdkconfig.h"
#include "freertos/projdefs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/spi_ll.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include "soc/soc.h"
#include "soc/spi_struct.h"

#include "Loop.h"
#include "MCP3564.h"

#define TAG "MCP3564"
/************************* MCP3564 REGISTER DEFS ******************************/

#define _ADCDATA_ 0x00                      //MCP3564 ADCDATA Register Address.        
#define _CONFIG0_ 0x01                      //MCP3564 CONFIG0 Register Address.
#define _CONFIG1_ 0x02                      //MCP3564 CONFIG1 Register Address.
#define _CONFIG2_ 0x03                      //MCP3564 CONFIG2 Register Address.
#define _CONFIG3_ 0x04                      //MCP3564 CONFIF3 Register Address.
#define _IRQ_ 0x05                          //MCP3564 IRQ Register Address.
#define _MUX_ 0x06                          //MCP3564 MUX Register Address.
#define _SCAN_ 0x07                         //MCP3564 SCAN Register Address.
#define _TIMER_ 0x08                        //MCP3564 TIMER Register Address.
#define _OFFSETCAL_ 0x09                    //MCP3564 OFFSETCAL Register Address.
#define _GAINCAL_ 0x0A                      //MCP3564 GAINCAL Register Address.
#define _RESERVED_B_ 0x0B                   //MCP3564 Reserved B Register Address.
#define _RESERVED_C_ 0x0C                   //MCP3564 Reserved C Register Address.
#define _LOCK_ 0x0D                         //MCP3564 LOCK Register Address.
#define _RESERVED_E_ 0x0E                   //MCP3564 Reserved E Register Address.
#define _CRCCFG_ 0x0F                       //MCP3564 CRCCFG Register Address.

#define _WRT_CTRL_ 0b01000010               //MCP3564 Write-CMD Command-Byte.
#define _RD_CTRL_  0b01000001               //MCP3564 Read-CMD Command-Byte.


// Macro to reverse endianness
#define INVERT_ENDIANNESS(x) (_Generic((x), \
    uint16_t: (((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00), \
    uint32_t: (((x) >> 24) & 0x000000FF) | \
              (((x) >>  8) & 0x0000FF00) | \
              (((x) <<  8) & 0x00FF0000) | \
              (((x) << 24) & 0xFF000000) \
))

static spi_device_handle_t spi;
uint32_t delta_time = 0;
int32_t EXT_RAM_BSS_ATTR queue_adc[QUEUE_ADC_LENGTH];
int32_t EXT_RAM_BSS_ATTR queue_hz[QUEUE_ADC_LENGTH];
int32_t pe_adc = 0, ps_adc = 0;
int32_t pe_hz = 0, ps_hz = 0;
bool flag_captura_adc=true;

///////////////////////
/// LOCAL FUNCTIONS ///
///////////////////////

static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg) {
    MCP3564_t* p = (MCP3564_t*)arg;

    if(flag_captura_adc) {
        queue_adc[pe_adc] = MCP3564_readVoltage(p);
        queue_hz[pe_adc] = delta_time;

        pe_adc = (pe_adc + 1) & (N_SAMPLES_MASK);
        pe_hz = (pe_hz + 1) & (N_SAMPLES_MASK);
    }
}

static uint32_t IRAM_ATTR fast_spi_trans(uint32_t data, size_t bitlen) {
	spi_ll_set_mosi_bitlen(&GPSPI2, bitlen);
    // Write data to be sent
    spi_ll_write_buffer(&GPSPI2, (const uint8_t*)&data, bitlen);

    // Trigger transmission
    spi_ll_enable_mosi(&GPSPI2, 1);
    spi_ll_enable_miso(&GPSPI2, 1);

    spi_ll_apply_config(&GPSPI2);
    spi_ll_user_start(&GPSPI2);

    // Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Read received data
    uint32_t data_in;
    spi_ll_read_buffer(&GPSPI2, (uint8_t*)&data_in, bitlen);

    return data_in;
}

static void init_spi(MCP3564_t* mcp_obj) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = mcp_obj->gpio_num_mosi,
        .miso_io_num = mcp_obj->gpio_num_miso,
        .sclk_io_num = mcp_obj->gpio_num_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .intr_flags = 0,
        .isr_cpu_id = 0
    };

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // can either be spi mode 0,0 or 1,1
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = mcp_obj->spi_clk_hz,  // 20 MHz clock speed
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
 
    spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_DISABLED);
    spi_bus_add_device(SPI2_HOST, &dev_config, &spi);  
    
    // Apply last configs via low-level lib
	const spi_ll_clock_val_t a = 33566787;
	
    spi_ll_master_init(&GPSPI2);
    spi_ll_master_set_clock_by_reg(&GPSPI2, &a);
    spi_ll_set_half_duplex(&GPSPI2, 0);
    
    spi_ll_master_set_cs_setup(&GPSPI2, 0);
    spi_ll_master_set_cs_hold(&GPSPI2, 1);
    spi_ll_master_select_cs(&GPSPI2, 5);
    
    spi_ll_clear_int_stat(&GPSPI2);
    spi_ll_set_dummy(&GPSPI2, 0);
    //spi_ll_set_mosi_bitlen(&GPSPI2, 32);
    spi_ll_set_addr_bitlen(&GPSPI2, 0);
    spi_ll_set_command_bitlen(&GPSPI2, 0);

    //gpio_set_direction(mcp_obj->gpio_num_ndrdy, GPIO_MODE_INPUT);
    gpio_set_direction(mcp_obj->gpio_num_cs, GPIO_MODE_OUTPUT);
    gpio_set_level (mcp_obj->gpio_num_cs, 1);
}

static void init_ledc(MCP3564_t* mcp_obj)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_1_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 20000000,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = mcp_obj->gpio_num_pwm,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void write_single_register(MCP3564_t* mcp_obj, uint8_t REG_ADDR, uint32_t WRT_DATA)
{
    uint32_t WRT_DATA_LOW = (WRT_DATA & 0x0000FF);              //Extract Low-Byte from 24-bit Write-Data.
    uint32_t WRT_DATA_HIGH = ((WRT_DATA & 0x00FF00) >> 8);      //Extract High-Byte from 24-bit Write-Data.
    uint32_t WRT_DATA_UPPER = ((WRT_DATA & 0xFF0000) >> 16);    //Extract Upper-Byte from 24-bit Write-Data.

	uint32_t WRT_CMD = (REG_ADDR << 2) | _WRT_CTRL_;
	uint32_t wrt_byte = 0;

    gpio_set_level(mcp_obj->gpio_num_cs, 0);
    
    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {
        wrt_byte = (uint16_t)((WRT_DATA_LOW<<8)|(WRT_CMD));
        fast_spi_trans((uint16_t)wrt_byte, 16);
    }
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {
        wrt_byte = (uint32_t)(WRT_DATA_LOW<<24)|(uint32_t)(WRT_DATA_HIGH<<16)|(uint32_t)(WRT_DATA_UPPER<<8)|(uint32_t)(WRT_CMD);
        fast_spi_trans(wrt_byte, 32);
    }
    
    gpio_set_level(mcp_obj->gpio_num_cs, 1);
}

static uint32_t read_single_register(MCP3564_t* mcp_obj, uint8_t REG_ADDR)
{
    uint32_t READ_VALUE = 0;
    uint8_t RD_CMD = (REG_ADDR << 2) | _RD_CTRL_;

    gpio_set_level(mcp_obj->gpio_num_cs, 0);
    
    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_) 
    {
        READ_VALUE = (uint32_t)fast_spi_trans(RD_CMD, 16);
        READ_VALUE = INVERT_ENDIANNESS((uint16_t)READ_VALUE) & 0x00FF;
    }
    else if (REG_ADDR == _RESERVED_E_ || REG_ADDR == _CRCCFG_) 
    {
        fast_spi_trans(RD_CMD, 8);
        READ_VALUE = (uint32_t)fast_spi_trans(RD_CMD, 16)>>8;
    }
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_)
    {
        READ_VALUE = fast_spi_trans(RD_CMD, 32);      
        READ_VALUE = INVERT_ENDIANNESS(READ_VALUE) & 0x00FFFFFF;
    }
    
    gpio_set_level(mcp_obj->gpio_num_cs, 1);
    
    return READ_VALUE;
}


////////////////////////
/// PUBLIC FUNCTIONS ///
////////////////////////

int32_t dequeue_adc() {
    if (ps_adc == pe_adc) return 0; // Queue is empty
    ps_adc = (ps_adc + 1) & (QUEUE_ADC_LENGTH - 1);
    return queue_adc[ps_adc];
}

int32_t dequeue_hz() {
    if (ps_hz == pe_hz) return queue_hz[ps_hz]; // Queue is empty
    ps_hz = (ps_hz + 1) & (QUEUE_ADC_LENGTH - 1);
    return queue_hz[ps_hz];
}

void MCP3564_startUp(MCP3564_t* mcp_obj) 
{   
    init_spi(mcp_obj);
    init_ledc(mcp_obj);

    ESP_LOGI(TAG, "set MCP configs");

    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input
    write_single_register(mcp_obj, _GAINCAL_, 0x800001);     
    ESP_LOGI(TAG, "GAINCAL = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _GAINCAL_));

    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative).
    write_single_register(mcp_obj,  _OFFSETCAL_, 0x000000);
    ESP_LOGI(TAG, "OFFSETCAL = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _OFFSETCAL_));

    //TIMER --> Disabled.
    write_single_register(mcp_obj,  _TIMER_, 0x000000);
    ESP_LOGI(TAG, "TIMER = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _TIMER_));

    //SCAN --> CH0 - CH5 --> ().
    write_single_register(mcp_obj, _SCAN_, 0x000F00);
    ESP_LOGI(TAG, "SCAN = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _SCAN_));

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    write_single_register(mcp_obj, _MUX_, 0xBC);
    ESP_LOGI(TAG, "MUX = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _MUX_));

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000100).
    write_single_register(mcp_obj, _IRQ_, 0x04);
    ESP_LOGI(TAG, "IRQ = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _IRQ_));

    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 32b + chid,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10110011). 
    write_single_register(mcp_obj, _CONFIG3_, 0x000000F3);
    ESP_LOGI(TAG, "CONFIG3 = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _CONFIG3_));
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 0 --> (0b10001011).
    write_single_register(mcp_obj, _CONFIG2_, 0x00000089);
    ESP_LOGI(TAG, "CONFIG2 = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _CONFIG2_));

    //CONFIG1 --> AMCLK = MCLK, OSR = 256 --> (0b00001100).      
    write_single_register(mcp_obj, _CONFIG1_, 0x00000000);
    ESP_LOGI(TAG, "CONFIG1 = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _CONFIG1_));

    //CONFIG0 --> VREF_SEL = extVOLT, CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b00010010).
    write_single_register(mcp_obj, _CONFIG0_, 0x00000013);
    ESP_LOGI(TAG, "CONFIG0 = 0x%08lX", (unsigned long)read_single_register(mcp_obj, _CONFIG0_));

	gpio_set_direction(mcp_obj->gpio_num_ndrdy, GPIO_MODE_INPUT);
    gpio_set_pull_mode(mcp_obj->gpio_num_ndrdy, GPIO_FLOATING);
    gpio_set_intr_type(mcp_obj->gpio_num_ndrdy, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(mcp_obj->gpio_num_ndrdy, GPIO_DRDY_IRQHandler, (void*)mcp_obj);   
}


uint32_t IRAM_ATTR MCP3564_readVoltage(MCP3564_t* mcp_obj) {
	gpio_set_level(mcp_obj->gpio_num_cs, 0);
	
	uint8_t data[5] = {_ADCDATA_<<2|_RD_CTRL_, 0x00, 0x00, 0x00, 0x00};
	
	spi_ll_set_mosi_bitlen(&GPSPI2, 40);
    // Write data to be sent
    spi_ll_write_buffer(&GPSPI2, (const uint8_t*)&data, 40);

    // Trigger transmission
    spi_ll_enable_mosi(&GPSPI2, 1);
    spi_ll_enable_miso(&GPSPI2, 1);

    spi_ll_apply_config(&GPSPI2);
    spi_ll_user_start(&GPSPI2);

    // Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Read received data
    uint8_t data_in[5];
    spi_ll_read_buffer(&GPSPI2, (uint8_t*)&data_in, 40);
	
	gpio_set_level(mcp_obj->gpio_num_cs, 1);
	
	return INVERT_ENDIANNESS(*(uint32_t *)&data_in[1]);
}
