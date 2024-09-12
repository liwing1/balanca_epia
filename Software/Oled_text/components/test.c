#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "soc/dport_reg.h"
#include "rom/ets_sys.h"

#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define SPI_PORT SPI2_HOST

#define MCP3564_CMD_RESET     0x38
#define MCP3564_CMD_READ_DATA 0x41

static const char *TAG = "MCP3564_READER";

void gpio_set_direction(int pin, int mode) {
    if (mode == 0) { // Input
        REG_WRITE(GPIO_ENABLE_W1TC_REG, BIT(pin));
    } else { // Output
        REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(pin));
    }
}

void gpio_set_level(int pin, int level) {
    if (level) {
        REG_WRITE(GPIO_OUT_W1TS_REG, BIT(pin));
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, BIT(pin));
    }
}

void spi_init() {
    // Enable SPI peripheral
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI2_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI2_RST);

    // Configure SPI pins
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MOSI], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MISO], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CLK], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CS], PIN_FUNC_GPIO);

    gpio_set_direction(PIN_NUM_MOSI, 1);
    gpio_set_direction(PIN_NUM_MISO, 0);
    gpio_set_direction(PIN_NUM_CLK, 1);
    gpio_set_direction(PIN_NUM_CS, 1);

    // Configure SPI parameters
    REG_WRITE(SPI_CLOCK_REG(SPI_PORT), SPI_CLK_EQU_SYSCLK_M); // Set clock divider
    REG_WRITE(SPI_USER_REG(SPI_PORT), SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN); // Set SPI mode
    REG_WRITE(SPI_USER1_REG(SPI_PORT), 0); // Set data bits to 0 (will be set later)
    REG_WRITE(SPI_SLAVE_REG(SPI_PORT), 0);
    REG_WRITE(SPI_PIN_REG(SPI_PORT), 0);
}

void spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, int len) {
    // Set CS low
    gpio_set_level(PIN_NUM_CS, 0);

    // Set data length
    REG_WRITE(SPI_MOSI_DLEN_REG(SPI_PORT), (len * 8) - 1);
    REG_WRITE(SPI_MISO_DLEN_REG(SPI_PORT), (len * 8) - 1);

    // Load TX data
    for (int i = 0; i < len; i += 4) {
        REG_WRITE(SPI_W0_REG(SPI_PORT) + i, *(uint32_t*)(tx_data + i));
    }

    // Start transmission
    REG_WRITE(SPI_CMD_REG(SPI_PORT), SPI_USR);

    // Wait for transmission to complete
    while (REG_READ(SPI_CMD_REG(SPI_PORT)) & SPI_USR);

    // Read RX data
    for (int i = 0; i < len; i += 4) {
        *(uint32_t*)(rx_data + i) = REG_READ(SPI_W0_REG(SPI_PORT) + i);
    }

    // Set CS high
    gpio_set_level(PIN_NUM_CS, 1);
}

void mcp3564_init() {
    spi_init();

    // Reset the MCP3564
    uint8_t cmd = MCP3564_CMD_RESET;
    spi_transfer(&cmd, NULL, 1);
    ets_delay_us(1000); // Wait for reset to complete
}

int32_t mcp3564_read_channel(uint8_t channel) {
    uint8_t tx_data[4] = {MCP3564_CMD_READ_DATA | channel, 0, 0, 0};
    uint8_t rx_data[4];
    
    spi_transfer(tx_data, rx_data, 4);

    int32_t result = (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    if (result & 0x800000) {
        result |= 0xFF000000; // Sign extend
    }
    return result;
}

void app_main(void) {
    mcp3564_init();

    while (1) {
        for (int i = 0; i < 6; i++) {
            int32_t adc_value = mcp3564_read_channel(i);
            ESP_LOGI(TAG, "Channel %d: %ld", i, adc_value);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Read every second
    }
}