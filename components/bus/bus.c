#include "bus.h"
#include "esp_log.h"

static const char *TAG = "BUS";

i2c_master_bus_handle_t i2c_bus = NULL;
spi_device_handle_t spi_lora = NULL;

static bool spi_initialized = false;

/* ================= I2C ================= */

esp_err_t bus_i2c_init(void)
{
    if (i2c_bus) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus));
    ESP_LOGI(TAG, "I2C bus initialized");

    return ESP_OK;
}

/* ================= SPI ================= */

esp_err_t bus_spi_init(void)
{
    if (spi_initialized) {
        ESP_LOGW(TAG, "SPI bus already initialized");
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_DISABLED)
    );

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LORA_CS,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_lora)
    );

    spi_initialized = true;

    ESP_LOGI(TAG, "SPI bus + LoRa device initialized");

    return ESP_OK;
}
