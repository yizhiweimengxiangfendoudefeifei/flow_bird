/**
 * vl53l1x.c - VL53L1X ToF sensor Ultra Lite Driver for ESP-IDF
 *
 * Based on ST's VL53L1X ULD (Ultra Lite Driver) API and Crazyflie zranger2.
 * Uses ESP-IDF I2C master driver with 16-bit register addressing.
 *
 * VL53L1X uses independent I2C bus (I2C_NUM_1, SDA=GPIO16, SCL=GPIO17).
 * VL53L1X address: 0x29 (7-bit)
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "vl53l1x.h"

static const char *TAG = "VL53L1X";

/* VL53L1X uses its own I2C bus */
#define VL53L1X_I2C_PORT   I2C_NUM_1
#define VL53L1X_SDA_IO     16
#define VL53L1X_SCL_IO     17
#define VL53L1X_I2C_FREQ   400000
#define VL53L1X_ADDR       VL53L1X_I2C_ADDR

/* Key register addresses (from ST ULD) */
#define VL53L1X_REG_SOFT_RESET                  0x0000
#define VL53L1X_REG_I2C_SLAVE_ADDR              0x0001
#define VL53L1X_REG_MODEL_ID                    0x010F
#define VL53L1X_REG_MODULE_TYPE                 0x0110
#define VL53L1X_REG_SYSTEM_STATUS               0x00E5
#define VL53L1X_REG_PAD_I2C_HV_EXTSUP_CONFIG   0x002E
#define VL53L1X_REG_GPIO_HV_MUX_CTRL           0x0030
#define VL53L1X_REG_GPIO_TIO_HV_STATUS         0x0031
#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR      0x0086
#define VL53L1X_REG_SYSTEM_MODE_START           0x0087
#define VL53L1X_REG_RESULT_RANGE_STATUS         0x0089
#define VL53L1X_REG_RESULT_FINAL_RANGE_MM       0x0096
#define VL53L1X_REG_RESULT_OSC_CALIBRATE_VAL    0x00DE
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI  0x005E
#define VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI  0x0061
#define VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_A       0x0060
#define VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_B       0x0063
#define VL53L1X_REG_RANGE_CONFIG_VALID_PHASE_HIGH     0x0069
#define VL53L1X_REG_SD_CONFIG_WOI_SD0                 0x0078
#define VL53L1X_REG_SD_CONFIG_INITIAL_PHASE_SD0       0x007A
#define VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD    0x006C
#define VL53L1X_REG_SYSTEM_THRESH_HIGH                0x0072
#define VL53L1X_REG_SYSTEM_THRESH_LOW                 0x0074

/* Expected IDs */
#define VL53L1X_MODEL_ID_VALUE    0xEA
#define VL53L1X_MODULE_TYPE_VALUE 0xCC

static bool vl53l1x_is_init = false;

/* ========== VL53L1X default configuration (91 bytes, from ST ULD) ========== */
/* Written starting at register 0x002D */
static const uint8_t vl53l1x_default_config[] = {
    0x00, /* 0x2D : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2E : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2F : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and target interrupt_polarity */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3A : not user-modifiable */
    0x00, /* 0x3B : not user-modifiable */
    0x00, /* 0x3C : not user-modifiable */
    0x00, /* 0x3D : not user-modifiable */
    0xFF, /* 0x3E : not user-modifiable */
    0x00, /* 0x3F : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0B, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4A : not user-modifiable */
    0x0A, /* 0x4B : not user-modifiable */
    0x21, /* 0x4C : not user-modifiable */
    0x00, /* 0x4D : not user-modifiable */
    0x00, /* 0x4E : not user-modifiable */
    0x05, /* 0x4F : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xC8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xFF, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5A : not user-modifiable */
    0x08, /* 0x5B : not user-modifiable */
    0x00, /* 0x5C : not user-modifiable */
    0x00, /* 0x5D : not user-modifiable */
    0x01, /* 0x5E : not user-modifiable */
    0xCC, /* 0x5F : not user-modifiable */
    0x0F, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xF1, /* 0x62 : not user-modifiable */
    0x0D, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma acum = 01 (default). Sigma = 0.25 degrees */
    0x68, /* 0x65 : Min count Rate 1Mcps (7.9 format). Default 1Mcps = 0x0080 */
    0x00, /* 0x66 : not user-modifiable */
    0x80, /* 0x67 : not user-modifiable */
    0x08, /* 0x68 : not user-modifiable */
    0xB8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6A : not user-modifiable */
    0x00, /* 0x6B : not user-modifiable */
    0x00, /* 0x6C : Intermeasurement period MSB. Default = 0x0001F4 = 500ms */
    0x00, /* 0x6D : Intermeasurement period */
    0x0F, /* 0x6E : Intermeasurement period */
    0x89, /* 0x6F : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB first) */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB  (in mm, MSB first) */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0F, /* 0x78 : not user-modifiable */
    0x0D, /* 0x79 : not user-modifiable */
    0x0E, /* 0x7A : not user-modifiable */
    0x0E, /* 0x7B : not user-modifiable */
    0x00, /* 0x7C : not user-modifiable */
    0x00, /* 0x7D : not user-modifiable */
    0x02, /* 0x7E : not user-modifiable */
    0xC7, /* 0x7F : ROI center */
    0xFF, /* 0x80 : XY ROI (Spad number) */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, 0x01=clear */
    0x00, /* 0x87 : start ranging, 0x40=start */
};

/* ========== Low-level I2C (16-bit register address) ========== */

static esp_err_t vl53l1x_write_byte(uint16_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);   /* reg MSB */
    i2c_master_write_byte(cmd, reg & 0xFF, true);          /* reg LSB */
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t vl53l1x_write_word(uint16_t reg, uint16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);  /* data MSB */
    i2c_master_write_byte(cmd, data & 0xFF, true);         /* data LSB */
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t vl53l1x_write_dword(uint16_t reg, uint32_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_write_byte(cmd, (data >> 24) & 0xFF, true);
    i2c_master_write_byte(cmd, (data >> 16) & 0xFF, true);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, data & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t vl53l1x_write_multi(uint16_t reg, const uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t vl53l1x_read_byte(uint16_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_start(cmd);  /* Repeated start */
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t vl53l1x_read_word(uint16_t reg, uint16_t *data)
{
    uint8_t buf[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(VL53L1X_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return ret;
}

/* ========== Public API ========== */

bool vl53l1x_init(void)
{
    if (vl53l1x_is_init) {
        return true;
    }

    /* Initialize I2C_NUM_1 for VL53L1X (SDA=GPIO16, SCL=GPIO17) */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = VL53L1X_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = VL53L1X_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = VL53L1X_I2C_FREQ,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(VL53L1X_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return false;
    }
    err = i2c_driver_install(VL53L1X_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "I2C_NUM_1 initialized (SDA=%d, SCL=%d)", VL53L1X_SDA_IO, VL53L1X_SCL_IO);

    /* Wait for device boot (check system status) */
    uint8_t boot_state = 0;
    int retries = 100;
    while (retries-- > 0) {
        if (vl53l1x_read_byte(VL53L1X_REG_SYSTEM_STATUS, &boot_state) == ESP_OK) {
            if (boot_state == 0x03 || boot_state == 0x01) {
                break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (retries <= 0) {
        ESP_LOGE(TAG, "VL53L1X boot timeout (status=0x%02X)", boot_state);
        return false;
    }

    /* Verify Model ID and Module Type */
    uint8_t model_id = 0, module_type = 0;
    vl53l1x_read_byte(VL53L1X_REG_MODEL_ID, &model_id);
    vl53l1x_read_byte(VL53L1X_REG_MODULE_TYPE, &module_type);
    ESP_LOGI(TAG, "Model ID: 0x%02X, Module Type: 0x%02X", model_id, module_type);

    if (model_id != VL53L1X_MODEL_ID_VALUE || module_type != VL53L1X_MODULE_TYPE_VALUE) {
        ESP_LOGE(TAG, "VL53L1X ID mismatch! Expected 0xEA/0xCC");
        return false;
    }

    /* Write the default configuration (91 bytes starting at 0x002D) */
    if (vl53l1x_write_multi(0x002D, vl53l1x_default_config,
                            sizeof(vl53l1x_default_config)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write default config");
        return false;
    }

    /* Set I2C to 2.8V mode (for most breakout boards) */
    uint8_t reg_val = 0;
    vl53l1x_read_byte(VL53L1X_REG_PAD_I2C_HV_EXTSUP_CONFIG, &reg_val);
    reg_val |= 0x01;  /* Set bit 0 for 2V8 mode */
    vl53l1x_write_byte(VL53L1X_REG_PAD_I2C_HV_EXTSUP_CONFIG, reg_val);

    /* Set default distance mode = MEDIUM (2m) and timing budget 50ms */
    vl53l1x_set_distance_mode(VL53L1X_DISTANCE_MODE_SHORT);
    vl53l1x_set_timing_budget(50);
    vl53l1x_set_inter_measurement(100);  /* 10 Hz */

    vl53l1x_is_init = true;
    ESP_LOGI(TAG, "VL53L1X initialized OK");
    return true;
}

bool vl53l1x_start_ranging(void)
{
    /* Clear interrupt then start */
    vl53l1x_clear_interrupt();
    return vl53l1x_write_byte(VL53L1X_REG_SYSTEM_MODE_START, 0x40) == ESP_OK;
}

bool vl53l1x_stop_ranging(void)
{
    return vl53l1x_write_byte(VL53L1X_REG_SYSTEM_MODE_START, 0x00) == ESP_OK;
}

bool vl53l1x_data_ready(void)
{
    uint8_t int_pol = 0;
    uint8_t gpio_status = 0;

    /* Read interrupt polarity from GPIO_HV_MUX_CTRL (bit 4) */
    vl53l1x_read_byte(VL53L1X_REG_GPIO_HV_MUX_CTRL, &int_pol);
    int_pol = (int_pol & 0x10) >> 4;  /* Active high or low */

    /* Read GPIO status */
    vl53l1x_read_byte(VL53L1X_REG_GPIO_TIO_HV_STATUS, &gpio_status);

    /* Data ready when GPIO matches opposite of configured polarity */
    return (gpio_status & 0x01) != int_pol;
}

bool vl53l1x_get_distance(uint16_t *range_mm)
{
    if (range_mm == NULL) return false;
    uint16_t val = 0;
    esp_err_t ret = vl53l1x_read_word(VL53L1X_REG_RESULT_FINAL_RANGE_MM, &val);
    *range_mm = val;
    return ret == ESP_OK;
}

bool vl53l1x_clear_interrupt(void)
{
    return vl53l1x_write_byte(VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) == ESP_OK;
}

bool vl53l1x_set_distance_mode(uint8_t mode)
{
    if (mode == VL53L1X_DISTANCE_MODE_SHORT) {
        /* Short mode: 1.3m max, better ambient light immunity */
        vl53l1x_write_byte(0x0060, 0x07);  /* VCSEL period A */
        vl53l1x_write_byte(0x0063, 0x05);  /* VCSEL period B */
        vl53l1x_write_byte(0x0069, 0x38);  /* Valid phase high */
        vl53l1x_write_byte(0x0078, 0x07);  /* WOI SD0 */
        vl53l1x_write_byte(0x0079, 0x05);  /* WOI SD1 */
        vl53l1x_write_byte(0x007A, 0x06);  /* Initial phase SD0 */
        vl53l1x_write_byte(0x007B, 0x06);  /* Initial phase SD1 */
    } else {
        /* Long mode: 4m max */
        vl53l1x_write_byte(0x0060, 0x0F);
        vl53l1x_write_byte(0x0063, 0x0D);
        vl53l1x_write_byte(0x0069, 0xB8);
        vl53l1x_write_byte(0x0078, 0x0F);
        vl53l1x_write_byte(0x0079, 0x0D);
        vl53l1x_write_byte(0x007A, 0x0E);
        vl53l1x_write_byte(0x007B, 0x0E);
    }
    return true;
}

bool vl53l1x_set_timing_budget(uint16_t budget_ms)
{
    /* Timing budget values for Short distance mode (from ST ULD) */
    /* Each entry: {timeout_macrop_a, timeout_macrop_b} for given budget */
    uint16_t a_val, b_val;
    switch (budget_ms) {
        case 15:  a_val = 0x001D; b_val = 0x0027; break;
        case 20:  a_val = 0x0051; b_val = 0x006E; break;
        case 33:  a_val = 0x00D6; b_val = 0x006E; break;
        case 50:  a_val = 0x01AE; b_val = 0x01E8; break;
        case 100: a_val = 0x02E1; b_val = 0x0388; break;
        case 200: a_val = 0x03E1; b_val = 0x0496; break;
        case 500: a_val = 0x0591; b_val = 0x05C1; break;
        default:  a_val = 0x01AE; b_val = 0x01E8; break; /* 50ms default */
    }
    vl53l1x_write_word(VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, a_val);
    vl53l1x_write_word(VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, b_val);
    return true;
}

bool vl53l1x_set_inter_measurement(uint32_t period_ms)
{
    /* The register value = period_ms * 1.055 (clock correction factor) */
    uint16_t osc_cal = 0;
    vl53l1x_read_word(VL53L1X_REG_RESULT_OSC_CALIBRATE_VAL, &osc_cal);

    uint32_t clock_pll;
    if (osc_cal != 0) {
        clock_pll = (uint32_t)((float)period_ms * (float)osc_cal);
    } else {
        /* Fallback: assume 1.055 factor */
        clock_pll = (uint32_t)((float)period_ms * 1.055f);
    }

    vl53l1x_write_dword(VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, clock_pll);
    return true;
}
