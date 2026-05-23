/**
 * vl53l1x.h - VL53L1X ToF sensor Ultra Lite Driver for ESP-IDF
 *
 * Simplified driver based on ST's VL53L1X ULD and Crazyflie zranger2 implementation.
 * Uses ESP-IDF I2C driver with 16-bit register addressing.
 */

#ifndef VL53L1X_H_
#define VL53L1X_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VL53L1X default 7-bit I2C address */
#define VL53L1X_I2C_ADDR        0x29

/* Distance modes */
#define VL53L1X_DISTANCE_MODE_SHORT   1   /* Up to 1.3m, better ambient immunity */
#define VL53L1X_DISTANCE_MODE_LONG    2   /* Up to 4m */

/**
 * Initialize the VL53L1X sensor.
 * Assumes I2C bus (I2C_NUM_0) is already initialized.
 *
 * @return true if initialization successful, false otherwise.
 */
bool vl53l1x_init(void);

/**
 * Start continuous ranging measurement.
 *
 * @return true on success.
 */
bool vl53l1x_start_ranging(void);

/**
 * Stop ranging measurement.
 *
 * @return true on success.
 */
bool vl53l1x_stop_ranging(void);

/**
 * Check if new measurement data is ready.
 *
 * @return true if data ready.
 */
bool vl53l1x_data_ready(void);

/**
 * Read the range measurement result.
 *
 * @param range_mm  Pointer to store the range in millimeters.
 * @return true on success.
 */
bool vl53l1x_get_distance(uint16_t *range_mm);

/**
 * Clear the data-ready interrupt to allow next measurement.
 *
 * @return true on success.
 */
bool vl53l1x_clear_interrupt(void);

/**
 * Set the distance mode.
 *
 * @param mode  VL53L1X_DISTANCE_MODE_SHORT or VL53L1X_DISTANCE_MODE_LONG
 * @return true on success.
 */
bool vl53l1x_set_distance_mode(uint8_t mode);

/**
 * Set the timing budget (measurement duration) in milliseconds.
 * Valid values: 15, 20, 33, 50, 100, 200, 500 ms.
 *
 * @param budget_ms  Timing budget in ms.
 * @return true on success.
 */
bool vl53l1x_set_timing_budget(uint16_t budget_ms);

/**
 * Set the inter-measurement period in milliseconds.
 * Must be >= timing budget. Determines ranging frequency.
 *
 * @param period_ms  Period in ms (e.g., 100 for 10Hz).
 * @return true on success.
 */
bool vl53l1x_set_inter_measurement(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1X_H_ */
