/**
 * pmw3901.h - PMW3901 optical flow sensor driver for ESP-IDF
 *
 * Adapted from Bitcraze/Crazyflie PMW3901 driver.
 */

#ifndef PMW3901_H_
#define PMW3901_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motionBurst_s {
    union {
        uint8_t motion;
        struct {
            uint8_t frameFrom0    : 1;
            uint8_t runMode       : 2;
            uint8_t reserved1     : 1;
            uint8_t rawFrom0      : 1;
            uint8_t reserved2     : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

/**
 * Initialize the PMW3901 sensor via SPI.
 *
 * @return true if initialization successful, false otherwise.
 */
bool pmw3901_init(void);

/**
 * Read current accumulated motion from PMW3901.
 *
 * @param motion  Pointer to motionBurst_t structure to fill with latest data.
 */
void pmw3901_read_motion(motionBurst_t *motion);

#ifdef __cplusplus
}
#endif

#endif /* PMW3901_H_ */
