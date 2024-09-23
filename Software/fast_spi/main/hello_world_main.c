#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_rom_sys.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/spi_ll.h"
#include "hal/gpio_types.h"
#include "soc/spi_struct.h"

// Define SPI peripheral and pins
#define SPI_HOST    SPI2
#define MOSI_PIN    GPIO_NUM_14
#define MISO_PIN	GPIO_NUM_11    
#define SCLK_PIN    GPIO_NUM_13
#define CS_PIN      GPIO_NUM_12
#define MCLK_PIN    GPIO_NUM_9
#define IRQ_PIN     GPIO_NUM_10

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


static spi_device_handle_t spi;

static void init_spi(void) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = GPIO_NUM_14,
        .miso_io_num = GPIO_NUM_11,
        .sclk_io_num = GPIO_NUM_13,
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
        .clock_speed_hz = 20000000,  // 20 MHz clock speed
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
    spi_ll_set_mosi_bitlen(&GPSPI2, 32);
    spi_ll_set_addr_bitlen(&GPSPI2, 0);
    spi_ll_set_command_bitlen(&GPSPI2, 0);
    
    gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CS_PIN, 1);
    
    gpio_set_direction(IRQ_PIN, GPIO_MODE_INPUT);
}


uint32_t normal_spi_trans(uint32_t data) {
	gpio_set_level(CS_PIN, 0);
	
	spi_transaction_t trans = {
		.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
		.length = 32,
		.tx_data = {(uint8_t)data, 0x00, 0x00, 0x00} 
	};
	
	spi_device_polling_transmit(spi, &trans);
	
	gpio_set_level(CS_PIN, 1);
	
	return *(uint32_t*)&trans.rx_data[0];
}

uint32_t IRAM_ATTR fast_spi_trans_32bit(uint32_t data) {
	// Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};
	
    // Write data to be sent
    spi_ll_write_buffer(&GPSPI2, (const uint8_t*)&data, 32);

    // Trigger transmission
    spi_ll_enable_mosi(&GPSPI2, 1);
    spi_ll_enable_miso(&GPSPI2, 1);

    spi_ll_apply_config(&GPSPI2);
    spi_ll_user_start(&GPSPI2);

    // Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Read received data
    uint32_t data_in;
    spi_ll_read_buffer(&GPSPI2, (uint8_t*)&data_in, 32);

    return data_in;
}

uint16_t IRAM_ATTR fast_spi_trans_16bit(uint16_t data) {
	// Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};
	
    // Write data to be sent
    spi_ll_write_buffer(&GPSPI2, (const uint8_t*)&data, 16);

    // Trigger transmission
    spi_ll_enable_mosi(&GPSPI2, 1);
    spi_ll_enable_miso(&GPSPI2, 1);

    spi_ll_apply_config(&GPSPI2);
    spi_ll_user_start(&GPSPI2);

    // Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Read received data
    uint16_t data_in;
    spi_ll_read_buffer(&GPSPI2, (uint8_t*)&data_in, 16);

    return data_in;
}

uint8_t IRAM_ATTR fast_spi_trans_8bit(uint8_t data) {
	// Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Write data to be sent
    spi_ll_write_buffer(&GPSPI2, (const uint8_t*)&data, 8);

    // Trigger transmission
    spi_ll_enable_mosi(&GPSPI2, 1);
    spi_ll_enable_miso(&GPSPI2, 1);

    spi_ll_apply_config(&GPSPI2);
    spi_ll_user_start(&GPSPI2);

    // Wait for transmission to complete
    while (spi_ll_get_running_cmd(&GPSPI2)) {};

    // Read received data
    uint8_t data_in;
    spi_ll_read_buffer(&GPSPI2, (uint8_t*)&data_in, 8);

    return data_in;
}

static void init_ledc(void)
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
        .gpio_num       = MCLK_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void writeSingleRegister(uint8_t REG_ADDR, uint32_t WRT_DATA)
{
    uint32_t WRT_DATA_LOW = (WRT_DATA & 0x0000FF);              //Extract Low-Byte from 24-bit Write-Data.
    uint32_t WRT_DATA_HIGH = ((WRT_DATA & 0x00FF00) >> 8);      //Extract High-Byte from 24-bit Write-Data.
    uint32_t WRT_DATA_UPPER = ((WRT_DATA & 0xFF0000) >> 16);    //Extract Upper-Byte from 24-bit Write-Data.

	uint32_t WRT_CMD = (REG_ADDR << 2) | _WRT_CTRL_;

    gpio_set_level(CS_PIN, 0);
    esp_rom_delay_us(2);
    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {   
        uint16_t test = (WRT_DATA_LOW<<8)|(WRT_CMD);
        uint16_t rx = fast_spi_trans_16bit(test);
        printf("test: %04x(%04x)\n", (unsigned)test, rx);
    }
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {   
        uint32_t test = (uint32_t)(WRT_DATA_LOW<<24)|(uint32_t)(WRT_DATA_HIGH<<16)|(uint32_t)(WRT_DATA_UPPER<<8)|(uint32_t)(WRT_CMD);
        fast_spi_trans_32bit(test);
        //printf("wrt test: %08lx\n", (unsigned long)test);
    }
    esp_rom_delay_us(2);
    gpio_set_level(CS_PIN, 1);
}

static uint32_t readSingleRegister(uint8_t REG_ADDR)
{
    uint32_t READ_VALUE = 0;
    uint8_t RD_CMD = (REG_ADDR << 2) | _RD_CTRL_;

    gpio_set_level(CS_PIN, 0);
    esp_rom_delay_us(2);
    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_) 
    {
        fast_spi_trans_8bit(RD_CMD);
        READ_VALUE = fast_spi_trans_8bit(0x00);
    }
    else if (REG_ADDR == _RESERVED_E_ || REG_ADDR == _CRCCFG_) 
    {
        fast_spi_trans_8bit(RD_CMD);
        READ_VALUE  = (uint32_t)fast_spi_trans_8bit(0x00) << 8;
        READ_VALUE |= (uint32_t)fast_spi_trans_8bit(0x00);
    }
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_)
    {
        fast_spi_trans_8bit(RD_CMD);
        READ_VALUE  = (uint32_t)fast_spi_trans_8bit(0x00) << 16;
        READ_VALUE |= (uint32_t)fast_spi_trans_8bit(0x00) << 8;
        READ_VALUE |= (uint32_t)fast_spi_trans_8bit(0x00);      
    }
    esp_rom_delay_us(2);
    gpio_set_level(CS_PIN, 1);
    return READ_VALUE;
}


static void MCP3564_startUp(void) 
{
    ESP_LOGI(TAG, "set MCP configs");

    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input
    writeSingleRegister(_GAINCAL_, 0x800001);     
    ESP_LOGI(TAG, "GAINCAL = 0x%08lX", (unsigned long)readSingleRegister(_GAINCAL_));

    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative).
    writeSingleRegister( _OFFSETCAL_, 0x000000);
    ESP_LOGI(TAG, "OFFSETCAL = 0x%08lX", (unsigned long)readSingleRegister(_OFFSETCAL_));

    //TIMER --> Disabled.
    writeSingleRegister( _TIMER_, 0x000000);
    ESP_LOGI(TAG, "TIMER = 0x%08lX", (unsigned long)readSingleRegister(_TIMER_));

    //SCAN --> CH0 - CH5 --> ().
    writeSingleRegister(_SCAN_, 0x00003F);
    ESP_LOGI(TAG, "SCAN = 0x%08lX", (unsigned long)readSingleRegister(_SCAN_));

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    writeSingleRegister(_MUX_, 0xBC);
    ESP_LOGI(TAG, "MUX = 0x%08lX", (unsigned long)readSingleRegister(_MUX_));

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000100).
    writeSingleRegister(_IRQ_, 0x04);
    ESP_LOGI(TAG, "IRQ = 0x%08lX", (unsigned long)readSingleRegister(_IRQ_));

    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 32b + chid,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10110011). 
    writeSingleRegister(_CONFIG3_, 0xF3);
    ESP_LOGI(TAG, "CONFIG3 = 0x%08lX", (unsigned long)readSingleRegister(_CONFIG3_));
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 0 --> (0b10001011).
    writeSingleRegister(_CONFIG2_, 0x89);
    ESP_LOGI(TAG, "CONFIG2 = 0x%08lX", (unsigned long)readSingleRegister(_CONFIG2_));

    //CONFIG1 --> AMCLK = MCLK, OSR = 256 --> (0b00001100).      
    writeSingleRegister(_CONFIG1_, 0x00);
    ESP_LOGI(TAG, "CONFIG1 = 0x%08lX", (unsigned long)readSingleRegister(_CONFIG1_));

    //CONFIG0 --> VREF_SEL = extVOLT, CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b00010010).
    writeSingleRegister(_CONFIG0_, 0x13);
    ESP_LOGI(TAG, "CONFIG0 = 0x%08lX", (unsigned long)readSingleRegister(_CONFIG0_));
}


void app_main(void) {
    printf("Starting SPI peripheral!\n");
    for(uint8_t i = 0; i < 5; i++) {
		printf("count %d\n", i);
    	vTaskDelay(pdMS_TO_TICKS(1000));	
	}
    
	init_spi();
	//MCP3564_startUp();
	//init_ledc();
	
	uint8_t gain = 0;
	while(1){
	    writeSingleRegister(_GAINCAL_, 0x800000+gain);     
    	ESP_LOGI(TAG, "GAINCAL = 0x%08lX", (unsigned long)readSingleRegister(_GAINCAL_));
    	gain++;
    	printf("\n");
		// First READ
		//gpio_set_level(CS_PIN, 0);
		//esp_rom_delay_us(2);
	 	//uint32_t value = fast_spi_trans_32bit(0x000000|(_GAINCAL_ << 2) | _RD_CTRL_);
	 	//esp_rom_delay_us(2);
	 	//gpio_set_level(CS_PIN, 1);
	 	//printf("read value: %lx\n", (unsigned long)value);
	 	
	 	//vTaskDelay(pdMS_TO_TICKS(1000));
	 	
 		//gpio_set_level(CS_PIN, 0);
		//esp_rom_delay_us(2);
		//uint32_t test = (0x02)<<24|(0x00)<<16|(0x80)<<8|((_GAINCAL_ << 2) | _WRT_CTRL_);
	 	//fast_spi_trans_32bit(test);
	 	//esp_rom_delay_us(2);
	 	//gpio_set_level(CS_PIN, 1);
	 	//printf("wrt test: %08lx\n", (unsigned long)test);
	 	

        vTaskDelay(pdMS_TO_TICKS(3000));
	}
}
