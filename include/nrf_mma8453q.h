/*
 * Copyright 2016 Fawaz Yemany, Uvis Egliens, Bart Monhemius.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @defgroup dd Driver Drivers
 * @file
 * @defgroup dd_accel Accelerometer Device Drivers
 * @{
 * @ingroup dd
 *
 * @brief Accelerometer device driver for the mma8453q.
 *
 */

#ifndef NRF_MMA8453Q_H
#define	NRF_MMA8453Q_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define REG_STATUS              0x00 /**<(R) Real time status*/
#define REG_OUT_X_MSB           0x01 /**<(R) [7:0] are 8 MSBs of 10-bit sample*/
#define REG_OUT_X_LSB           0x02 /**<(R) [7:6] are 2 LSBs of 10-bit sample*/
#define REG_OUT_Y_MSB           0x03 /**<(R) [7:0] are 8 MSBs of 10-bit sample*/
#define REG_OUT_Y_LSB           0x04 /**<(R) [7:6] are 2 LSBs of 10-bit sample*/
#define REG_OUT_Z_MSB           0x05 /**<(R) [7:0] are 8 MSBs of 10-bit sample*/
#define REG_OUT_Z_LSB           0x06 /**<(R) [7:6] are 2 LSBs of 10-bit sample*/
#define REG_SYSMOD              0x0b /**<(R) Current system mode*/
#define REG_INT_SOURCE          0x0c /**<(R) Interrupt status*/
#define REG_WHO_AM_I            0x0d /**<(R) Device ID (0x3A)*/
#define REG_XYZ_DATA_CFG        0x0e /**<(R/W) Dynamic range settings*/
#define REG_HP_FILTER_CUTOFF    0x0f /**<(R/W) cut-off frequency is set to 16Hz @ 800Hz*/
#define REG_PL_STATUS           0x10 /**<(R) Landscape/Portrait orientation status*/
#define REG_PL_CFG              0x11 /**<(R/W) Landscape/Portrait configuration*/
#define REG_PL_COUNT            0x12 /**<(R) Landscape/Portrait debounce counter*/
#define REG_PL_BF_ZCOMP         0x13 /**<(R) Back-Front, Z-Lock trip threshold*/
#define REG_P_L_THS_REG         0x14 /**<(R/W) Portrait to Landscape trip angle is 29 degree*/
#define REG_FF_MT_CFG           0x15 /**<(R/W) Freefall/motion functional block configuration*/
#define REG_FF_MT_SRC           0x16 /**<(R) Freefall/motion event source register*/
#define REG_FF_MT_THS           0x17 /**<(R/W) Freefall/motion threshold register*/
#define REG_FF_MT_COUNT         0x18 /**<(R/W) Freefall/motion debounce counter*/
#define REG_TRANSIENT_CFG       0x1d /**<(R/W) Transient functional block configuration*/
#define REG_TRANSIENT_SRC       0x1e /**<(R) Transient event status register*/
#define REG_TRANSIENT_THS       0x1f /**<(R/W) Transient event threshold*/
#define REG_TRANSIENT_COUNT     0x20 /**<(R/W) Transient debounce counter*/
#define REG_PULSE_CFG           0x21 /**<(R/W) ELE, Double_XYZ or Single_XYZ*/
#define REG_PULSE_SRC           0x22 /**<(R) EA, Double_XYZ or Single_XYZ*/
#define REG_PULSE_THSX          0x23 /**<(R/W) X pulse threshold*/
#define REG_PULSE_THSY          0x24 /**<(R/W) Y pulse threshold*/
#define REG_PULSE_THSZ          0x25 /**<(R/W) Z pulse threshold*/
#define REG_PULSE_TMLT          0x26 /**<(R/W) Time limit for pulse*/
#define REG_PULSE_LTCY          0x27 /**<(R/W) Latency time for 2nd pulse*/
#define REG_PULSE_WIND          0x28 /**<(R/W) Window time for 2nd pulse*/
#define REG_ASLP_COUNT          0x29 /**<(R/W) Counter setting for auto-sleep*/
#define REG_CTRL_REG1           0x2a /**<(R/W) ODR = 800 Hz, STANDBY mode*/
#define REG_CTRL_REG2           0x2b /**<(R/W) Sleep enable, OS Modes, RST, ST*/
#define REG_CTRL_REG3           0x2c /**<(R/W) Wake from sleep, IPOL, PP_OD*/
#define REG_CTRL_REG4           0x2d /**<(R/W) Interrupt enable register*/
#define REG_CTRL_REG5           0x2e /**<(R/W) Interrupt pin (INT1/INT2) map*/
#define REG_OFF_X               0x2f /**<(R/W) X-axis offset adjust*/
#define REG_OFF_Y               0x30 /**<(R/W) Y-axis offset adjust*/
#define REG_OFF_Z               0x31 /**<(R/W) Z-axis offset adjust*/
#define FULL_SCALE_RANGE_2g     0x00
#define FULL_SCALE_RANGE_4g     0x01
#define FULL_SCALE_RANGE_8g     0x02
#define ACTIVE_MASK             1
#define RES_MASK                2
#define DATA_RATE_MASK          0xC7
#define DATA_RATE_800           0
#define DATA_RATE_400           1
#define DATA_RATE_200           2
#define DATA_RATE_100           3
#define DATA_RATE_50            4

#define DRV_TWI_CONF_DEFAULT (drv_accelConfig_t) {      \
        .sdaPin = TWI0_CONFIG_SDA,                      \
        .sclPin = TWI0_CONFIG_SCL,                      \
        .twiFreq = TWI0_CONFIG_FREQUENCY,               \
        .id = TWI0_INSTANCE_INDEX,                      \
        .enable = true                                  \
    }

typedef struct drv_accelHandle *drv_accelHandle_t;

typedef struct {
    uint32_t sdaPin;                        /**<Pin to use for the TWI data pin*/
    uint32_t sclPin;                        /**<Pin to use for the TWI clock pin*/
    nrf_twi_frequency_t twiFreq;            /**<TWI clock frequency*/
    uint8_t id;                             /**<TWI module ID, not being used right now*/
    bool enable;                            /**<If the TWI hardware should be powered on on initialization or not*/
    nrf_drv_twi_evt_handler_t twiHandler;   /**<Handle event from the TWI driver in non-blocking mode, leave NULL for blocking*/
} drv_twiConfig_t;

typedef struct {
    uint8_t gRange;                 /**<Range in G forces, either 2,4 or 8*/
    bool highRes;                   /**<If 10-bits mode should be used rather then 8-bits*/
    uint8_t samplingRate;           /**<The sampling rate, see DATA_RATE_x*/
    uint8_t address;                /**<TWI Device address*/
} drv_accelConfig_t;

typedef struct {
    uint16_t x;                     /**<X value of the accelerometer*/
    uint16_t y;                     /**<Y value of the accelerometer*/
    uint16_t z;                     /**<Z value of the accelerometer*/
    bool failed : 1;                /**<If reading failed or not */
} drv_accelData_t;

/**
 * @brief Initialize the accelerometer.
 * @details This initializes the TWI hardware.
 *
 * @param conf Configuration for TWI.
 * @return A handle to the accelerometer and TWI driver.
 */
drv_accelHandle_t drv_accelInit(drv_twiConfig_t *conf);

/**
 * @brief Enable the TWI hardware
 * @details This disables the power of the TWI hardware. This can be useful
 * to save power.
 * @param handle Handle to the accelerometer driver to turn off.
 */
void drv_accelEnable(drv_accelHandle_t handle);

/**
 * @brief Set the accelerometer configuration.
 * @details This function can configure the accelerometer itself. For available
 * options, see drv_accelConfig_t.
 *
 * @param handle Accelerometer handle to configure.
 * @param conf Configuration to apply.
 *
 * @retval  true If the configuration was successful.
 * @retval  false If a TWI error occurred.
 */
bool drv_accelConfigure(drv_accelHandle_t handle, drv_accelConfig_t *conf);

/**
 * @brief Read the x, y and z value from the accelerometer.
 *
 * @param handle [Handle to the accelerometer to read.
 * @return A structure containing the data read from the accelerometer. If the
 * failed member is true, an error in the communication occurred.
 */
drv_accelData_t drv_accelRead(drv_accelHandle_t handle);

/**
 * @brief Disable the TWI hardware.
 * @details [This turns off the power from the TWI hardware. This can be useful
 * to save power.
 *
 * @param handle Handle to the accelerometer to turn off.
 */
void drv_accelDisable(drv_accelHandle_t handle);

#ifdef	__cplusplus
}
#endif

#endif	/* NRF_MMA8453Q_H */

/**
 *@}
 **/
