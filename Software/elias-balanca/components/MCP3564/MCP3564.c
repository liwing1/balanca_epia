#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
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

#define nPOR_STATUS     BIT0
#define nCRCCFG_STATUS  BIT1

#define LEDC_DUTY               (1) // Set duty to 50%. (2 ** 1) * 50% = 4096
#define LEDC_FREQUENCY          (12000000) // Frequency in Hertz. Set frequency at 4 kHz

static spi_device_handle_t spi;

static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg)
{
    uint32_t gpio_num = ((MCP3564_t*)arg)->gpio_num_ndrdy;
    if(gpio_num) ((MCP3564_t*)arg)->flag_drdy++;
}

static void example_ledc_init(MCP3564_t* mcp_obj)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_1_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
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

    // Set duty to 50%
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_DUTY);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void init_spi(MCP3564_t* mcp_obj)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = mcp_obj->gpio_num_mosi,
        .miso_io_num = mcp_obj->gpio_num_miso,
        .sclk_io_num = mcp_obj->gpio_num_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // can either be spi mode 0,0 or 1,1
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_CLOCK_SPEED,  // 1 MHz clock speed
        .spics_io_num = mcp_obj->gpio_num_cs,
        .flags = 0,
        .queue_size = 16,
        .pre_cb = NULL,
        .post_cb = NULL
    };
 
    spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &dev_config, &spi);    
}

static void writeSingleRegister(MCP3564_t* mcp_obj, uint8_t WRT_CMD, uint32_t WRT_DATA)
{
    spi_transaction_t trans = {0};
    trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    uint8_t WRT_DATA_LOW;                                  //Register-Data Low-Byte.
    uint8_t WRT_DATA_HIGH;                                 //Register-Data High-Byte.
    uint8_t WRT_DATA_UPPER;                                //Register-Data Upper-Byte.
    uint8_t REG_ADDR;

    WRT_DATA_LOW = (uint8_t) (WRT_DATA & 0x0000FF);              //Extract Low-Byte from 24-bit Write-Data.
    WRT_DATA_HIGH = (uint8_t) ((WRT_DATA & 0x00FF00) >> 8);      //Extract High-Byte from 24-bit Write-Data.
    WRT_DATA_UPPER = (uint8_t) ((WRT_DATA & 0xFF0000) >> 16);    //Extract Upper-Byte from 24-bit Write-Data.

    REG_ADDR = (WRT_CMD & 0b00111100) >> 2;                 //Extract MCP3564 Register-Address for Write-CMD

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {
        trans.length = 16;
        trans.tx_data[0] = WRT_CMD;
        trans.tx_data[1] = WRT_DATA_LOW;
        spi_device_transmit(spi, &trans);
    } 
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {
        trans.length = 32;
        trans.tx_data[0] = WRT_CMD;
        trans.tx_data[1] = WRT_DATA_UPPER;
        trans.tx_data[2] = WRT_DATA_HIGH;
        trans.tx_data[3] = WRT_DATA_LOW;
        spi_device_transmit(spi, &trans);
    }
}

static uint32_t readSingleRegister(MCP3564_t* mcp_obj, uint8_t RD_CMD)
{
    spi_transaction_t trans = {0};
    trans.flags = SPI_TRANS_USE_TXDATA|SPI_TRANS_USE_RXDATA;

    uint32_t READ_VALUE = 0;
    uint8_t REG_ADDR;

    REG_ADDR = (RD_CMD & 0b00111100) >> 2;                  //Extract MCP3564 Register-Address for Write-CMD

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_) 
    {                  
        trans.length = 16;
        trans.tx_data[0] = RD_CMD;
        trans.tx_data[1] = 0x00;
        spi_device_transmit(spi, &trans);
        READ_VALUE = (uint32_t)trans.rx_data[1];
    } 
    else if (REG_ADDR == _RESERVED_E_ || REG_ADDR == _CRCCFG_) 
    {
        trans.length = 24;
        trans.tx_data[0] = RD_CMD;
        trans.tx_data[1] = 0x00;
        trans.tx_data[2] = 0x00;
        spi_device_transmit(spi, &trans);
        READ_VALUE  = (uint32_t)trans.rx_data[1] << 8;
        READ_VALUE |= (uint32_t)trans.rx_data[2];
    }
    else if (REG_ADDR == _ADCDATA_ || REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_) 
    {
        uint8_t snd_data[5] = {0x00};
        snd_data[0] = RD_CMD;
        uint8_t rcv_data[5] = {0x00};
        trans.length = 8 * 5;
        trans.flags = 0;
        trans.tx_buffer = snd_data;
        trans.rx_buffer = rcv_data;
        spi_device_transmit(spi, &trans);

        READ_VALUE  = ((uint32_t)rcv_data[1]) << 24;
        READ_VALUE |= ((uint32_t)rcv_data[2]) << 16;
        READ_VALUE |= ((uint32_t)rcv_data[3]) << 8;
        READ_VALUE |= ((uint32_t)rcv_data[4]);
    }
    return READ_VALUE;
}

int32_t MCP3564_signExtend(uint32_t Bytes)
{
    int32_t signByte    = ((int32_t) (Bytes & 0x0F000000));
    int32_t dataBytes   = ((int32_t) (Bytes & 0x00FFFFFF));

    return ((signByte<<4) | signByte | dataBytes);
}

void MCP3564_startUp(MCP3564_t* mcp_obj) 
{
    init_spi(mcp_obj);

    // First enable interrupts
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1 << mcp_obj->gpio_num_ndrdy,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&gpio_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(mcp_obj->gpio_num_ndrdy, GPIO_DRDY_IRQHandler, (void*)mcp_obj);

    ESP_LOGI(TAG, "set MCP configs");

    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input
    writeSingleRegister(mcp_obj, ((_GAINCAL_ << 2) | _WRT_CTRL_), 0x800000);     
    ESP_LOGI(TAG, "GAINCAL = %lx", readSingleRegister(mcp_obj, ((_GAINCAL_ << 2) | _RD_CTRL_)));

    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative).
    writeSingleRegister(mcp_obj, ((_OFFSETCAL_ << 2) | _WRT_CTRL_), 0x000000);
    ESP_LOGI(TAG, "OFFSETCAL = %lx", readSingleRegister(mcp_obj, ((_OFFSETCAL_ << 2) | _RD_CTRL_)));

    //TIMER --> Disabled.
    writeSingleRegister(mcp_obj, ((_TIMER_ << 2) | _WRT_CTRL_), 0x000000);
    ESP_LOGI(TAG, "TIMER = %lx", readSingleRegister(mcp_obj, ((_TIMER_ << 2) | _RD_CTRL_)));

    //SCAN --> CH0 - CH5 --> ().
    writeSingleRegister(mcp_obj, ((_SCAN_ << 2) | _WRT_CTRL_), 0x00003F);
    ESP_LOGI(TAG, "SCAN = %lx", readSingleRegister(mcp_obj, ((_SCAN_ << 2) | _RD_CTRL_)));

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    writeSingleRegister(mcp_obj, ((_MUX_ << 2) | _WRT_CTRL_), 0xBC);
    ESP_LOGI(TAG, "MUX = %lx", readSingleRegister(mcp_obj, ((_MUX_ << 2) | _RD_CTRL_)));

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000100).
    writeSingleRegister(mcp_obj, ((_IRQ_ << 2) | _WRT_CTRL_), 0x04);
    ESP_LOGI(TAG, "IRQ = %lx", readSingleRegister(mcp_obj, ((_IRQ_ << 2) | _RD_CTRL_)));

    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 32b + chid,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10110011). 
    writeSingleRegister(mcp_obj, ((_CONFIG3_ << 2) | _WRT_CTRL_), 0xB3);
    ESP_LOGI(TAG, "CONFIG3 = %lx", readSingleRegister(mcp_obj, ((_CONFIG3_ << 2) | _RD_CTRL_)));
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 0 --> (0b10001011).
    writeSingleRegister(mcp_obj, ((_CONFIG2_ << 2) | _WRT_CTRL_), 0x89);
    ESP_LOGI(TAG, "CONFIG2 = %lx", readSingleRegister(mcp_obj, ((_CONFIG2_ << 2) | _RD_CTRL_)));

    //CONFIG1 --> AMCLK = MCLK, OSR = 256 --> (0b00001100).      
    writeSingleRegister(mcp_obj, ((_CONFIG1_ << 2) | _WRT_CTRL_), 0x0C);
    ESP_LOGI(TAG, "CONFIG1 = %lx", readSingleRegister(mcp_obj, ((_CONFIG1_ << 2) | _RD_CTRL_)));

    //CONFIG0 --> VREF_SEL = extVOLT, CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b00010010).
    writeSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0x12);
    ESP_LOGI(TAG, "CONFIG0 = %lx", readSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _RD_CTRL_)));

    ESP_LOGI(TAG, "Start MCLK");
    example_ledc_init(mcp_obj);
}

void MCP3564_startConversion(MCP3564_t* mcp_obj)
{
    writeSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0x13);
}

uint32_t MCP3564_readVoltage(MCP3564_t* mcp_obj)
{
    return readSingleRegister(mcp_obj, (_ADCDATA_ << 2) | _RD_CTRL_);
}

uint32_t MCP3564_readChannels(MCP3564_t* mcp_obj, float* buffer)
{
    uint32_t timeout_us = 100;
    uint32_t time_us = 0;
    MCP3564_startConversion(mcp_obj);
    uint32_t list[8] = {0};

    uint32_t column;
    for(uint8_t i = 0; i < 6; i++)
    {
        while(!mcp_obj->flag_drdy) 
        {
            esp_rom_delay_us(50);//vTaskDelay(1); // To do: add timeout
            if(time_us+=50 >= timeout_us) return 1;
        }
        
        uint32_t readV = MCP3564_readVoltage(mcp_obj);
        list[i] = readV;
        if(readV)
        {
            column = ((readV & 0xF0000000) >> 7 * 4);
            buffer[column] = (float)MCP3564_signExtend(readV) * 3.3/8388608.0;
        }
        mcp_obj->flag_drdy = 0;             //Data-Ready Flag via MCP3564 IRQ Alert.
    }

    for(uint8_t i = 0; i < 6; i++) {
        printf("%lX\t", list[i]);
    }
    printf("\n");

    return 0;
}
