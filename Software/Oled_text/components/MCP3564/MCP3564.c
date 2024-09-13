#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "MCP3564.h"
#include "..\..\main\wim_ios.h"
#include "esp_system.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_mem_struct.h"
#include "driver/gpio.h"
#include "soc/spi_reg.h"
#include "soc/dport_reg.h"

#define PIN_NUM_MISO ADC_SDI
#define PIN_NUM_MOSI ADC_SDO
#define PIN_NUM_CLK  ADC_SCK
#define PIN_NUM_CS   ADC_CS

#define SPI_HOST    SPI2_HOST

static const char *TAG_SPI = "SPI";
static const char *TAG_MCP = "MCP3564";

//cria buffer com 1024 elementos de 32 bits na memoria externa ram

int32_t queue_adc[QUEUE_ADC_LENGTH];

//extern uint32_t queue_adc;

int32_t pe_adc = 0, ps_adc = 0;
bool flag_captura_adc=true;

// SPI hardware instance
//static spi_dev_t *const SPI = &GPSPI2;

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


typedef union{
    uint8_t bytes[5];
    struct{
        // uint8_t status;
        uint8_t sign:4;
        uint8_t channel:4;
        uint8_t upper;
        uint8_t high;
        uint8_t low;
    };
} adc_format_t;

uint32_t freq = 0;

static TaskHandle_t task_handle;
static spi_device_handle_t spi;
static const uint8_t snd_data[5] = {(_ADCDATA_ << 2) | _RD_CTRL_, 0x00, 0x00, 0x00, 0x00};
static uint8_t* rcv_data;
static spi_transaction_t adc_trans = {
    .length = 5*8,
    .tx_buffer = snd_data
};

//EXT_RAM_BSS_ATTR uint32_t history[N_SAMPLES][N_CHANNELS];

//void MCP3564_spiHandle(void* p);
void MCP3564_spiHandle();
void test_task(void* p);

// static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg)
// {
//     spi_device_queue_trans(spi, &adc_trans, portMAX_DELAY);
// }

static void Clock_ledc_init()
{

//#define CLOCK_FREQ_HZ 20000000 // 20 MHz
#define CLOCK_FREQ_HZ 6000000 //  MHz
    ESP_LOGI(TAG_SPI, "Configuring LEDC timer for clock generation");

    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT, // For 50% duty cycle
        .timer_num = LEDC_TIMER_0,
        .freq_hz = CLOCK_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = ADC_MCLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1, // 50% duty cycle for 1-bit resolution
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG_SPI, "LEDC configuration complete. 20MHz clock started on GPIO %d", ADC_MCLK);
}

#if 0
static void init_spi(void)
{
    // Reset SPI peripheral
    SPI->user.val = 0;
    SPI->ctrl.val = 0;

    // Configure SPI mode
    SPI->user.usr_mosi = 1;
    SPI->user.usr_miso = 1;
    SPI->user.doutdin = 1;
    SPI->user.sio = 0;
    SPI->slave.slave_mode = 0;

    // Configure clock
    SPI->clock.clk_equ_sysclk = 0;
    SPI->clock.clkcnt_n = 3;  // 40MHz / 4 = 10MHz
    SPI->clock.clkcnt_h = 1;
    SPI->clock.clkcnt_l = 3;

    // Configure pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_MOSI) | (1ULL << PIN_NUM_CLK) | (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << PIN_NUM_MISO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // Connect SPI signals using GPIO matrix
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_NUM_MOSI], PIN_FUNC_GPIO);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_NUM_MISO], PIN_FUNC_GPIO);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_NUM_CLK], PIN_FUNC_GPIO);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_NUM_CS], PIN_FUNC_GPIO);

    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);

    esp_rom_gpio_connect_out_signal(PIN_NUM_MOSI, GPSPI2_MOSI_IDX, false, false);
    esp_rom_gpio_connect_in_signal(PIN_NUM_MISO, GPSPI2_MISO_IDX, false);
    esp_rom_gpio_connect_out_signal(PIN_NUM_CLK, GPSPI2_CLK_IDX, false, false);
}

#else
static void init_spi(void)
{
    //  ------------------ Setup do ADC MCP3564R -----------------------------------

#define SPI_CLOCK_FREQ_HZ 20000000

    spi_bus_config_t bus_config = {
        .mosi_io_num = ADC_SDI,
        .miso_io_num = ADC_SDO,
        .sclk_io_num = ADC_SCK,
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
        .clock_speed_hz = SPI_CLOCK_FREQ_HZ,
        .spics_io_num = ADC_CS,
        .flags = 0,
        .queue_size = 256,
        .pre_cb = NULL,
        .post_cb = NULL
    };
 
    spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &dev_config, &spi);  
  
}

#endif

static void writeSingleRegister(uint8_t WRT_CMD, uint32_t WRT_DATA)
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

static uint32_t readSingleRegister(uint8_t RD_CMD)
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
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_) 
    {
        trans.length = 32;
        trans.tx_data[0] = RD_CMD;
        trans.tx_data[1] = 0x00;
        trans.tx_data[2] = 0x00;
        trans.tx_data[3] = 0x00;
        spi_device_transmit(spi, &trans);
        READ_VALUE  = (uint32_t)trans.rx_data[1] << 16;
        READ_VALUE |= (uint32_t)trans.rx_data[2] << 8;
        READ_VALUE |= (uint32_t)trans.rx_data[3];       
    }    
    else if (REG_ADDR == _ADCDATA_) 
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

#define MCP3564_CMD_READ_DATA 0x41

int32_t readDataRegister()
{

    uint8_t tx_data[1] = {MCP3564_CMD_READ_DATA};
    uint8_t rx_data[5] = {0x00};

    spi_transaction_t trans = {
        .length = 8 * 5, // 5 bytes
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    // Set CS low
    gpio_set_level(PIN_NUM_CS, 0);

    // Transmit the command and receive the data
    spi_device_transmit(spi, &trans);

    // Set CS high
    gpio_set_level(PIN_NUM_CS, 1);

    // Combine the received bytes into a 32-bit value
    int32_t READ_VALUE = ((uint32_t)rx_data[1]) << 24;
    READ_VALUE |= ((uint32_t)rx_data[2]) << 16;
    READ_VALUE |= ((uint32_t)rx_data[3]) << 8;
    READ_VALUE |= ((uint32_t)rx_data[4]);

    return READ_VALUE;
}


void MCP3564_startUp(void) 
{
    init_spi();
//    *mcp_obj->history = (uint32_t*)&history;

    ESP_LOGI(TAG_MCP, "set MCP configs");

    //GAINCAL --> 0
    writeSingleRegister(((_GAINCAL_ << 2) | _WRT_CTRL_), 0x800000);     
    ESP_LOGI(TAG_MCP, "GAINCAL = %lx", readSingleRegister(((_GAINCAL_ << 2) | _RD_CTRL_)));

    //OFFSETCAL --> 0
    writeSingleRegister(((_OFFSETCAL_ << 2) | _WRT_CTRL_), 0x000000);
    ESP_LOGI(TAG_MCP, "OFFSETCAL = %lx", readSingleRegister(((_OFFSETCAL_ << 2) | _RD_CTRL_)));

    //TIMER --> Disabled.
    writeSingleRegister(((_TIMER_ << 2) | _WRT_CTRL_), 0x000000);
    ESP_LOGI(TAG_MCP, "TIMER = %lx", readSingleRegister(((_TIMER_ << 2) | _RD_CTRL_)));

    //SCAN --> CH0 - CH5 --> ().
    //writeSingleRegister(((_SCAN_ << 2) | _WRT_CTRL_), 0x00F003);
    //writeSingleRegister(((_SCAN_ << 2) | _WRT_CTRL_), 0x00003F);

    // Só dois canais por enquanto

    writeSingleRegister(((_SCAN_ << 2) | _WRT_CTRL_), 0x00003F);    
    ESP_LOGI(TAG_MCP, "SCAN = %lx", readSingleRegister(((_SCAN_ << 2) | _RD_CTRL_)));

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    writeSingleRegister(((_MUX_ << 2) | _WRT_CTRL_), 0xBC);
    ESP_LOGI(TAG_MCP, "MUX = %lx", readSingleRegister(((_MUX_ << 2) | _RD_CTRL_)));

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000100).
    writeSingleRegister(((_IRQ_ << 2) | _WRT_CTRL_), 0x04);
    ESP_LOGI(TAG_MCP, "IRQ = %lx", readSingleRegister(((_IRQ_ << 2) | _RD_CTRL_)));

    //CONFIG3 --> continuous conversion cycle in SCAN mode, FORMAT = 32b + chid,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Disabled, GAINCAL = Disabled --> (0b11110000). 
    writeSingleRegister(((_CONFIG3_ << 2) | _WRT_CTRL_), 0xF0);
    ESP_LOGI(TAG_MCP, "CONFIG3 = %lx", readSingleRegister(((_CONFIG3_ << 2) | _RD_CTRL_)));
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 0 --> (0b10001011).
    writeSingleRegister(((_CONFIG2_ << 2) | _WRT_CTRL_), 0x89);
    ESP_LOGI(TAG_MCP, "CONFIG2 = %lx", readSingleRegister(((_CONFIG2_ << 2) | _RD_CTRL_)));

    //CONFIG1 --> AMCLK = MCLK, OSR = 32 --> (0b00000000).   
    writeSingleRegister(((_CONFIG1_ << 2) | _WRT_CTRL_), 0x00);

    ESP_LOGI(TAG_MCP, "CONFIG1 = %lx", readSingleRegister(((_CONFIG1_ << 2) | _RD_CTRL_)));

    //CONFIG0 --> VREF_SEL = extVOLT, CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Conversion Mode --> (0b00010011).
    writeSingleRegister(((_CONFIG0_ << 2) | _WRT_CTRL_), 0x13);
    ESP_LOGI(TAG_MCP, "CONFIG0 = %lx", readSingleRegister(((_CONFIG0_ << 2) | _RD_CTRL_)));

    rcv_data = (uint8_t*)heap_caps_calloc(1, 5, MALLOC_CAP_DMA);
    adc_trans.rx_buffer = rcv_data;

    xTaskCreatePinnedToCore(MCP3564_spiHandle, "MCP3564_spiHandle", 2048*4, NULL, configMAX_PRIORITIES-1, &task_handle, APP_CPU_NUM);

      gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1 << ADC_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_cfg);

    //xTaskCreatePinnedToCore(MCP3564_spiHandle, "MCP3564_spiHandle", 2048*4, (void*)mcp_obj, configMAX_PRIORITIES-1, &task_handle, APP_CPU_NUM);
    //gpio_install_isr_service(0);
    //gpio_isr_handler_add(mcp_obj->gpio_num_ndrdy, GPIO_DRDY_IRQHandler, (void*)mcp_obj);

    ESP_LOGI(TAG_MCP, "Start MCLK");
    Clock_ledc_init();

}



int32_t dequeue_adc() {
    if (ps_adc == pe_adc) return 0; // Queue is empty
    ps_adc = (ps_adc + 1) & (QUEUE_ADC_LENGTH - 1);
    return queue_adc[ps_adc];
}

// SPI transaction structure
//spi_transaction_t trans2;
//extern spi_dev_t SPI2;

#define SPI_NUM                 2  // Using SPI2 (SPI2_HOST)
#define SPI_BASE                DR_REG_SPI2_BASE

void MCP3564_spiHandle()
{

    adc_format_t* format;
    //MCP3564_t* mcp_obj = (MCP3564_t*)p;

    spi_transaction_t adc_trans = {
        .flags = SPI_TRANS_CS_KEEP_ACTIVE | SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .length = 8 // First byte addr
    };
    adc_trans.tx_data[0] = (_ADCDATA_ << 2) | _RD_CTRL_;

    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);

    // Start polling ADC_DATA reg with cs low
    spi_device_polling_transmit(spi, &adc_trans);

    // Setup new trans to retrieve data
    adc_trans.tx_data[0] = 0x00;
    adc_trans.length = 4 * 8;
    spi_device_polling_transmit(spi, &adc_trans);

    //while (gpio_get_level(ADC_IRQ) == 0);

    while (1)
    {

        if (flag_captura_adc)  { 

            //se o DRDY for 0, espera ele ir para 1
            //while (gpio_get_level(ADC_IRQ) == 0);
            //espera o DRDY ir para 0
            while (gpio_get_level(ADC_IRQ) == 1);


        gpio_set_level(LED_YELLOW,0);   
           
        spi_device_polling_transmit(spi, &adc_trans);
        
        gpio_set_level(LED_YELLOW,1); 

        format = (adc_format_t*)adc_trans.rx_data;

        //    int32_t dado = readDataRegister();
        //((uint32_t)format->sign   << 4 | (uint32_t)format->sign) << 24 |         

        // int32_t dado = 
        // ((uint32_t)format->channel) << 28|
        // ((uint32_t)format->sign)    << 24|
        // ((uint32_t)format->upper)   << 16| 
        // ((uint32_t)format->high)    << 8 | 
        // ((uint32_t)format->low);

        int32_t dado = 
        ((uint32_t)format->bytes[0]) << 24|
        ((uint32_t)format->bytes[1]) << 16|
        ((uint32_t)format->bytes[2]) <<  8| 
        ((uint32_t)format->bytes[3]);


        //int32_t dado = 0x04123456 | ((pe_adc%6)<<28);

     
            //int32_t dado = pe_adc;


            // Poe na Fila
            pe_adc = (pe_adc + 1) & (QUEUE_ADC_LENGTH - 1);
            // if (pe_adc == ps_adc) {
            //     pe_adc = (pe_adc - 1) & (QUEUE_ADC_LENGTH - 1);
            // }
            queue_adc[pe_adc] = (uint32_t)dado;

            // ps_adc=(pe_adc+1)&(QUEUE_ADC_LENGTH-1);
        
        }
        
    }    
    
}

#if 0
bool is_row_filled(uint32_t* row, size_t row_size) {
    for(size_t i = 0; i < row_size; i++) {
        if(row[i] == 0) {
            return false;
        }
    }
    return true;
}

void MCP3564_spiHandle(void *p)
{
    adc_format_t* format = (adc_format_t*)rcv_data;
    MCP3564_t* mcp_obj = (MCP3564_t*)p;
    spi_transaction_t* rx_trans;

    while (1)
    {
        while(spi_device_get_trans_result(spi, &rx_trans, portMAX_DELAY) == ESP_OK)
        {
            history[mcp_obj->flag_drdy][format->channel] = \
            ((uint32_t)format->sign   << 4 | (uint32_t)format->sign) << 24 | \
            ((uint32_t)format->upper) << 16| \
            ((uint32_t)format->high)  << 8 | \
            ((uint32_t)format->low);

            freq++;

            if(is_row_filled(history[mcp_obj->flag_drdy], N_CHANNELS)) {
                mcp_obj->flag_drdy = (mcp_obj->flag_drdy + 1) & (N_SAMPLES);
            }
        }
    }
}
#endif