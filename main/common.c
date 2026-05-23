/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <esp32/rom/ets_sys.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/gpio.h>
#include "bmp3.h"
#include "common.h"

#define I2C_MASTER_SDA_IO (gpio_num_t)21
#define I2C_MASTER_SCL_IO (gpio_num_t)22
#define I2C_MASTER_FREQ_HZ 400000

#define SPI_MASTER_MISO_IO 22
#define SPI_MASTER_MOSI_IO 23
#define SPI_MASTER_CLK_IO 21
#define SPI_MASTER_CS_IO 5
spi_device_handle_t spi_device;

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID 0xD3

/* Variable to store the device address */
static uint8_t dev_addr;

/***************************************************************************************************************
i2c master initialization
****************************************************************************************************************/
esp_err_t rtrobot_i2c_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/***************************************************************************************************************
spi master initialization
****************************************************************************************************************/
esp_err_t rtrobot_spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MASTER_MISO_IO,
        .mosi_io_num = SPI_MASTER_MOSI_IO,
        .sclk_io_num = SPI_MASTER_CLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0, // SPI mode 0
        .address_bits = 8,
        .spics_io_num = SPI_MASTER_CS_IO, // CS pin
        .queue_size = 7                   // We want to be able to queue 7 transactions at a time
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 0);

    // Attach the device to the SPI bus
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi_device);
    return ret;
}

/*!
 * I2C read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    if (length == 0)
        return 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    vTaskDelay(15 / portTICK_PERIOD_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1)
        i2c_master_read(cmd, reg_data, length - 1, (i2c_ack_type_t)0x0);
    i2c_master_read_byte(cmd, reg_data + length - 1, (i2c_ack_type_t)0x01);
    i2c_master_stop(cmd);
    int8_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

/*!
 * I2C write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);

    i2c_master_stop(cmd);
    int8_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*!
 * SPI read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = reg_addr | 0x80;
    transaction.length = (uint16_t)length * 8;
    transaction.rxlength = (uint16_t)length * 8;
    transaction.tx_buffer = NULL;
    transaction.user = NULL;
    transaction.rx_buffer = reg_data;
    int8_t err = spi_device_transmit(spi_device, &transaction);
    return err;
}

/*!
 * SPI write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = reg_addr & 0x7f;
    transaction.length = (uint16_t)length * 8;
    transaction.rxlength = 0;
    transaction.tx_buffer = reg_data;
    transaction.rx_buffer = NULL;
    transaction.user = NULL;
    int8_t err = spi_device_transmit(spi_device, &transaction);
    return err;
}

/*!
 * Delay function map to COINES platform
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // ets_delay_us(period);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
    case BMP3_OK:

        /* Do nothing */
        break;
    case BMP3_E_NULL_PTR:
        printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
        break;
    case BMP3_E_COMM_FAIL:
        printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
        break;
    case BMP3_E_INVALID_LEN:
        printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
        break;
    case BMP3_E_DEV_NOT_FOUND:
        printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
        break;
    case BMP3_E_CONFIGURATION_ERR:
        printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
        break;
    case BMP3_W_SENSOR_NOT_ENABLED:
        printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
        break;
    case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
        printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
        break;
    default:
        printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
        break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
{
    int8_t rslt = BMP3_OK;

    /* Bus configuration : I2C */
    if (intf == BMP3_I2C_INTF)
    {
        printf("I2C Interface\n");
        rtrobot_i2c_init();
        dev_addr = BMP3_ADDR_I2C_SEC;
        bmp3->read = bmp3_i2c_read;
        bmp3->write = bmp3_i2c_write;
        bmp3->intf = BMP3_I2C_INTF;
    }
    /* Bus configuration : SPI */
    else if (intf == BMP3_SPI_INTF)
    {
        printf("SPI Interface\n");
        rtrobot_spi_init();
        bmp3->read = bmp3_spi_read;
        bmp3->write = bmp3_spi_write;
        bmp3->intf = BMP3_SPI_INTF;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    bmp3->delay_us = bmp3_delay_us;
    bmp3->intf_ptr = &dev_addr;

    return rslt;
}
