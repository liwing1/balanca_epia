#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOSConfig.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "MCP3564.h"
#include "Loop.h"


#define NUM_SAMPLES         (32768) //1<<15
#define NUM_SAMPLES_MASK    (NUM_SAMPLES-1)

#define SPI_CLK_HZ          20000000
#define IRQ_PIN             GPIO_NUM_10
#define BTN_PIN             GPIO_NUM_38


uint32_t EXT_RAM_BSS_ATTR adc_queue[NUM_SAMPLES];
MCP3564_t MCP_instance = {
    .gpio_num_pwm       = GPIO_NUM_9,
    .gpio_num_ndrdy     = GPIO_NUM_10,
    .gpio_num_miso      = GPIO_NUM_11,
    .gpio_num_cs        = GPIO_NUM_12,
    .gpio_num_clk       = GPIO_NUM_13,
    .gpio_num_mosi      = GPIO_NUM_14,
    .spi_clk_hz         = SPI_CLK_HZ,
    .flag_drdy = 0,
};


static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg) {
    MCP3564_t* p = (MCP3564_t*)arg;

    adc_queue[p->flag_drdy] = MCP3564_readVoltage(p);

    p->flag_drdy = (p->flag_drdy + 1) & NUM_SAMPLES_MASK;
}


void app_main(void) {	
    MCP3564_startUp(&MCP_instance);

    gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_PIN, GPIO_PULLUP_ONLY);

	gpio_set_direction(IRQ_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IRQ_PIN, GPIO_FLOATING);
    gpio_set_intr_type(IRQ_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(IRQ_PIN, GPIO_DRDY_IRQHandler, (void*)&MCP_instance);   

    Init_loop();
	
	while(1) {
        uint32_t dt = dequeue();
        if(dt) {
            printf("dt: %ld\n", dt);
        }

		if(gpio_get_level(BTN_PIN) == 0) {
			
			for(uint8_t i = 0; i < 10; i++) {
				int32_t volts = (adc_queue[i] & 0x0F000000)<<4 | (adc_queue[i] & 0x0FFFFFFF);
				printf("%f, ", volts*(3.3/(1<<23)));
			}
			printf("%lu\n", (unsigned long)esp_log_timestamp());

            vTaskDelay(pdMS_TO_TICKS(1000));
		}
		
        vTaskDelay(pdMS_TO_TICKS(10));
	}
}