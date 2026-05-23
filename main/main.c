/*
 * main.c:
 * Copyright (c) 2014-2019 Rtrobot. <admin@rtrobot.org>
 *  <http://rtrobot.org>
 ***********************************************************************
 */

#include <stdio.h>
#include <math.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "common.h"
#include "pmw3901.h"
#include "vl53l1x.h"


/* Scan the I2C bus and print all responding device addresses (7-bit). */
static void i2c_scan(void)
{
    printf("Scanning I2C bus...\n");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            printf("  Found device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0)
        printf("No I2C devices found. Check wiring/pullups/power/CSB/SDO.\n");
    else
        printf("Scan done, %u device(s) found.\n", found);
}

void i2c_bmp3_task(void *pvParameters)
{
    int8_t rslt = 0;
    uint8_t loop = 0;
    uint8_t settings_sel;
    struct bmp3_dev dev;
    struct bmp3_data data = {0};
    struct bmp3_settings settings = {0};
    struct bmp3_status status = {0};

    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    /* Scan the bus once after I2C driver is installed to confirm the sensor address. */
    i2c_scan();

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
        {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

#ifdef BMP3_FLOAT_COMPENSATION
            {
                float altitude = pressure_to_altitude((float)data.pressure);
                printf("Data[%d]  T: %.2f deg C, P: %.2f Pa, Alt: %.2f m\n",
                       loop, (data.temperature), (data.pressure), altitude);
            }
#else
            {
                /* data.pressure is in Pa*100 (fixed-point), convert to Pa */
                float pressure_pa = (float)((uint32_t)data.pressure) / 100.0f;
                float altitude = pressure_to_altitude(pressure_pa);
                printf("Data[%d]  T: %ld deg C, P: %lu Pa, Alt: %.2f m\n",
                       loop, (long int)(int32_t)(data.temperature / 100),
                       (long unsigned int)(uint32_t)(data.pressure / 100), altitude);
            }
#endif

            loop = loop + 1;
        }
    }
}


/* ======================= PMW3901 Position Estimation ======================= */

/* PMW3901 sensor geometry */
#define FLOW_NPIX          30.0f       /* 30x30 pixel sensor */
#define FLOW_FOV_RAD       0.71674f    /* ~42 deg field of view */
#define FLOW_RAD_PER_PIX   (FLOW_FOV_RAD / FLOW_NPIX)

/* Assumed height above ground (meters). Adjust based on actual mounting. */
#define DEFAULT_HEIGHT_M   0.1f

/* Outlier removal threshold (pixels per read) */
#define OUTLIER_LIMIT      100

/* IIR low-pass filter coefficient for velocity (0~1, higher = more smoothing) */
#define VEL_LP_ALPHA       0.7f

static const char *TAG_FLOW = "PMW3901_POS";

static void pmw3901_position_task(void *pvParameters)
{
    /* Wait for sensor power-up */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (!pmw3901_init()) {
        ESP_LOGE(TAG_FLOW, "PMW3901 init failed, task exiting.");
        vTaskDelete(NULL);
        return;
    }

    /* Position estimation state */
    float est_x = 0.0f;   /* Estimated X position (m) */
    float est_y = 0.0f;   /* Estimated Y position (m) */
    float est_vx = 0.0f;  /* Filtered X velocity (m/s) */
    float est_vy = 0.0f;  /* Filtered Y velocity (m/s) */
    float height = DEFAULT_HEIGHT_M;

    motionBurst_t motion;
    int64_t last_time_us = esp_timer_get_time();

    ESP_LOGI(TAG_FLOW, "PMW3901 position estimation started (height=%.2f m)", height);
    printf("\n%-10s %-10s %-10s %-10s %-8s %-8s\n",
           "X(m)", "Y(m)", "Vx(m/s)", "Vy(m/s)", "dX(px)", "dY(px)");

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);  /* ~10 Hz */

        pmw3901_read_motion(&motion);

        /* Compute dt in seconds */
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;

        if (dt <= 0.0f) continue;

        /* Raw pixel displacements (flip axes to match body frame if needed) */
        int16_t dpx = -motion.deltaY;  /* sensor X -> body X */
        int16_t dpy = -motion.deltaX;  /* sensor Y -> body Y */

        /* Outlier removal */
        if (abs(dpx) >= OUTLIER_LIMIT || abs(dpy) >= OUTLIER_LIMIT) {
            continue;
        }

        /* Only process if motion flag valid (0xB0 indicates good data) */
        if (motion.motion != 0xB0) {
            continue;
        }

        /*
         * Convert pixel displacement to linear velocity:
         *   angular_rate = (dpixel * rad_per_pixel) / dt
         *   linear_velocity = angular_rate * height
         */
        float omega_x = ((float)dpx * FLOW_RAD_PER_PIX) / dt;
        float omega_y = ((float)dpy * FLOW_RAD_PER_PIX) / dt;

        float vx_raw = height * omega_x;
        float vy_raw = height * omega_y;

        /* IIR low-pass filter */
        est_vx = VEL_LP_ALPHA * est_vx + (1.0f - VEL_LP_ALPHA) * vx_raw;
        est_vy = VEL_LP_ALPHA * est_vy + (1.0f - VEL_LP_ALPHA) * vy_raw;

        /* Dead-reckon position */
        est_x += est_vx * dt;
        est_y += est_vy * dt;

        /* Print estimated position and velocity */
        printf("X: %7.4f m | Y: %7.4f m | Vx: %6.3f m/s | Vy: %6.3f m/s | dX: %4d px | dY: %4d px\n",
               est_x, est_y, est_vx, est_vy, dpx, dpy);
    }
}

/* ======================= VL53L1X Height Estimation ======================= */

/* IIR filter coefficient for height (0~1, higher = more trust in previous estimate) */
#define HEIGHT_EST_ALPHA    0.90f

/* Maximum valid range in mm (outlier rejection) */
#define RANGE_OUTLIER_LIMIT_MM  5000

static const char *TAG_TOF = "VL53L1X_ALT";

static void vl53l1x_height_task(void *pvParameters)
{
    /* Wait for I2C bus to be initialized by BMP3 task */
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG_TOF, "Initializing VL53L1X...");

    if (!vl53l1x_init()) {
        ESP_LOGE(TAG_TOF, "VL53L1X init failed, task exiting.");
        vTaskDelete(NULL);
        return;
    }

    /* Start continuous ranging */
    if (!vl53l1x_start_ranging()) {
        ESP_LOGE(TAG_TOF, "VL53L1X start ranging failed.");
        vTaskDelete(NULL);
        return;
    }

    float estimated_z = 0.0f;  /* Filtered height estimate (m) */
    bool first_measurement = true;

    ESP_LOGI(TAG_TOF, "VL53L1X height estimation started (10Hz)");

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);  /* 10 Hz */

        /* Wait for data ready */
        if (!vl53l1x_data_ready()) {
            continue;
        }

        /* Read distance */
        uint16_t range_mm = 0;
        if (!vl53l1x_get_distance(&range_mm)) {
            vl53l1x_clear_interrupt();
            continue;
        }

        /* Clear interrupt for next measurement */
        vl53l1x_clear_interrupt();

        /* Outlier rejection */
        if (range_mm == 0 || range_mm > RANGE_OUTLIER_LIMIT_MM) {
            continue;
        }

        /* Convert mm to meters */
        float distance_m = (float)range_mm * 0.001f;

        /* IIR filter for height estimation
         * (Reference: position_estimator_altitude.c)
         * filteredZ = alpha * estimatedZ + (1-alpha) * measurement
         */
        if (first_measurement) {
            estimated_z = distance_m;
            first_measurement = false;
        } else {
            estimated_z = HEIGHT_EST_ALPHA * estimated_z +
                          (1.0f - HEIGHT_EST_ALPHA) * distance_m;
        }

        printf("[ToF] Range: %u mm | Height(est): %.3f m\n", range_mm, estimated_z);
    }
}

void app_main()
{
    nvs_flash_init();
    xTaskCreate(i2c_bmp3_task, "i2c_BMP3_task", 4096, NULL, 5, NULL);
    xTaskCreate(pmw3901_position_task, "pmw3901_pos_task", 4096, NULL, 5, NULL);
    xTaskCreate(vl53l1x_height_task, "vl53l1x_alt_task", 4096, NULL, 5, NULL);
}
