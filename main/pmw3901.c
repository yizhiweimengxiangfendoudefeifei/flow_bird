/**
 * pmw3901.c - PMW3901 optical flow sensor driver for ESP-IDF
 *
 * Uses ESP-IDF spi_master driver. Adapted from Bitcraze/Crazyflie PMW3901 driver.
 * SPI pins (directly usable on ESP32 VSPI):
 *   MOSI: GPIO 12
 *   MISO: GPIO 14
 *   CLK:  GPIO 26
 *   CS:   GPIO 27
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pmw3901.h"

static const char *TAG = "PMW3901";

/* ----- SPI Pin Configuration ----- */
#define PMW3901_SPI_HOST   VSPI_HOST
#define PMW3901_MISO_IO    15
#define PMW3901_MOSI_IO    13
#define PMW3901_CLK_IO     26
#define PMW3901_CS_IO      27

/* SPI clock: PMW3901 max is 2 MHz */
#define PMW3901_SPI_FREQ_HZ  (2 * 1000 * 1000)

static spi_device_handle_t pmw_spi_dev;
static bool pmw_is_init = false;

/* ========== Low-level SPI helpers ========== */

static void pmw3901_cs_low(void)
{
    gpio_set_level(PMW3901_CS_IO, 0);
}

static void pmw3901_cs_high(void)
{
    gpio_set_level(PMW3901_CS_IO, 1);
}

static void delay_us(uint32_t us)
{
    uint64_t start = (uint64_t)esp_timer_get_time();
    while (((uint64_t)esp_timer_get_time() - start) < us);
}

/**
 * Write one byte over SPI (full-duplex, ignore received data).
 */
static void spi_write_byte(uint8_t data)
{
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &data;
    spi_device_transmit(pmw_spi_dev, &t);
}

/**
 * Read one byte over SPI (send 0x00, capture response).
 */
static uint8_t spi_read_byte(void)
{
    uint8_t rx = 0;
    uint8_t tx = 0x00;
    spi_transaction_t t = {0};
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &tx;
    t.rx_buffer = &rx;
    spi_device_transmit(pmw_spi_dev, &t);
    return rx;
}

/* ========== PMW3901 Register Access ========== */

static void register_write(uint8_t reg, uint8_t value)
{
    reg |= 0x80u;  /* MSB=1 for write */

    pmw3901_cs_low();
    delay_us(50);
    spi_write_byte(reg);
    delay_us(50);
    spi_write_byte(value);
    delay_us(50);
    pmw3901_cs_high();
    delay_us(200);
}

static uint8_t register_read(uint8_t reg)
{
    reg &= ~0x80u;  /* MSB=0 for read */

    pmw3901_cs_low();
    delay_us(50);
    spi_write_byte(reg);
    delay_us(500);
    uint8_t data = spi_read_byte();
    delay_us(50);
    pmw3901_cs_high();
    delay_us(200);

    return data;
}

/* ========== PMW3901 Init Registers (from datasheet) ========== */

static void init_registers(void)
{
    register_write(0x7F, 0x00);
    register_write(0x61, 0xAD);
    register_write(0x7F, 0x03);
    register_write(0x40, 0x00);
    register_write(0x7F, 0x05);
    register_write(0x41, 0xB3);
    register_write(0x43, 0xF1);
    register_write(0x45, 0x14);
    register_write(0x5B, 0x32);
    register_write(0x5F, 0x34);
    register_write(0x7B, 0x08);
    register_write(0x7F, 0x06);
    register_write(0x44, 0x1B);
    register_write(0x40, 0xBF);
    register_write(0x4E, 0x3F);
    register_write(0x7F, 0x08);
    register_write(0x65, 0x20);
    register_write(0x6A, 0x18);
    register_write(0x7F, 0x09);
    register_write(0x4F, 0xAF);
    register_write(0x5F, 0x40);
    register_write(0x48, 0x80);
    register_write(0x49, 0x80);
    register_write(0x57, 0x77);
    register_write(0x60, 0x78);
    register_write(0x61, 0x78);
    register_write(0x62, 0x08);
    register_write(0x63, 0x50);
    register_write(0x7F, 0x0A);
    register_write(0x45, 0x60);
    register_write(0x7F, 0x00);
    register_write(0x4D, 0x11);
    register_write(0x55, 0x80);
    register_write(0x74, 0x1F);
    register_write(0x75, 0x1F);
    register_write(0x4A, 0x78);
    register_write(0x4B, 0x78);
    register_write(0x44, 0x08);
    register_write(0x45, 0x50);
    register_write(0x64, 0xFF);
    register_write(0x65, 0x1F);
    register_write(0x7F, 0x14);
    register_write(0x65, 0x67);
    register_write(0x66, 0x08);
    register_write(0x63, 0x70);
    register_write(0x7F, 0x15);
    register_write(0x48, 0x48);
    register_write(0x7F, 0x07);
    register_write(0x41, 0x0D);
    register_write(0x43, 0x14);
    register_write(0x4B, 0x0E);
    register_write(0x45, 0x0F);
    register_write(0x44, 0x42);
    register_write(0x4C, 0x80);
    register_write(0x7F, 0x10);
    register_write(0x5B, 0x02);
    register_write(0x7F, 0x07);
    register_write(0x40, 0x41);
    register_write(0x70, 0x00);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    register_write(0x32, 0x44);
    register_write(0x7F, 0x07);
    register_write(0x40, 0x40);
    register_write(0x7F, 0x06);
    register_write(0x62, 0xF0);
    register_write(0x63, 0x00);
    register_write(0x7F, 0x0D);
    register_write(0x48, 0xC0);
    register_write(0x6F, 0xD5);
    register_write(0x7F, 0x00);
    register_write(0x5B, 0xA0);
    register_write(0x4E, 0xA8);
    register_write(0x5A, 0x50);
    register_write(0x40, 0x80);

    register_write(0x7F, 0x00);
    register_write(0x5A, 0x10);
    register_write(0x54, 0x00);
}

/* ========== Public API ========== */

bool pmw3901_init(void)
{
    if (pmw_is_init) {
        return true;
    }

    esp_err_t ret;

    /* Configure CS pin as GPIO output */
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << PMW3901_CS_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    pmw3901_cs_high();

    /* Initialize SPI bus */
    spi_bus_config_t bus_cfg = {
        .miso_io_num = PMW3901_MISO_IO,
        .mosi_io_num = PMW3901_MOSI_IO,
        .sclk_io_num = PMW3901_CLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    ret = spi_bus_initialize(PMW3901_SPI_HOST, &bus_cfg, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Attach PMW3901 device (CS managed manually) */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = PMW3901_SPI_FREQ_HZ,
        .mode = 3,               /* SPI Mode 3 (CPOL=1, CPHA=1) for PMW3901 */
        .spics_io_num = -1,      /* CS controlled manually */
        .queue_size = 1,
    };

    ret = spi_bus_add_device(PMW3901_SPI_HOST, &dev_cfg, &pmw_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return false;
    }

    vTaskDelay(40 / portTICK_PERIOD_MS);

    /* Power-on sequence: toggle CS */
    pmw3901_cs_high();
    vTaskDelay(2 / portTICK_PERIOD_MS);
    pmw3901_cs_low();
    vTaskDelay(2 / portTICK_PERIOD_MS);
    pmw3901_cs_high();
    vTaskDelay(2 / portTICK_PERIOD_MS);

    /* Verify chip ID */
    uint8_t chip_id = register_read(0x00);
    uint8_t inv_chip_id = register_read(0x5F);

    ESP_LOGI(TAG, "Chip ID: 0x%02X, Inverse ID: 0x%02X", chip_id, inv_chip_id);

    if (chip_id == 0x49 || inv_chip_id == 0xB6) {
        /* Power-on reset */
        register_write(0x3A, 0x5A);
        vTaskDelay(5 / portTICK_PERIOD_MS);

        /* Read motion registers once to clear */
        register_read(0x02);
        register_read(0x03);
        register_read(0x04);
        register_read(0x05);
        register_read(0x06);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        /* Write performance optimization registers */
        init_registers();

        pmw_is_init = true;
        ESP_LOGI(TAG, "PMW3901 initialized successfully");
    } else {
        ESP_LOGE(TAG, "PMW3901 not detected! Check wiring.");
    }

    return pmw_is_init;
}

void pmw3901_read_motion(motionBurst_t *motion)
{
    if (!pmw_is_init || motion == NULL) {
        return;
    }

    uint8_t address = 0x16;
    uint8_t buf[sizeof(motionBurst_t)] = {0};

    pmw3901_cs_low();
    delay_us(50);
    spi_write_byte(address);
    delay_us(50);

    /* Read burst data byte by byte */
    for (size_t i = 0; i < sizeof(motionBurst_t); i++) {
        buf[i] = spi_read_byte();
    }

    delay_us(50);
    pmw3901_cs_high();
    delay_us(50);

    memcpy(motion, buf, sizeof(motionBurst_t));

    /* Fix shutter byte order (big-endian from sensor) */
    uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
    realShutter |= (motion->shutter & 0x0FF) << 8;
    motion->shutter = realShutter;
}
