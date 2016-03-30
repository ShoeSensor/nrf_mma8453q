/*
 * Copyright 2016 Fawaz Yemany, Uvis Egliens.
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

#ifndef NRF_MMA8453Q_H
#define	NRF_MMA8453Q_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define DRV_ACCEL_CONF_DEFAULT (drv_accelConfig_t) {    \
        .sdaPin = TWI0_CONFIG_SDA,                      \
        .sclPin = TWI0_CONFIG_SCL,                      \
        .twiFreq = TWI0_CONFIG_FREQUENCY,               \
        .id = TWI0_INSTANCE_INDEX                       \
    }

typedef struct drv_accelHandle *drv_accelHandle_t;

typedef struct {
    uint32_t sdaPin;
    uint32_t sclPin;
    nrf_twi_frequency_t twiFreq;
    uint8_t id;
} drv_accelConfig_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    bool failed : 1;
} drv_accelData_t;

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param conf [description]
 * @return [description]
 */
drv_accelHandle_t drv_accelInit(drv_accelConfig_t *conf);

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param handle [description]
 * @param adress [description]
 * @param data [description]
 * @param value [description]
 * @return [description]
 */
bool drv_accelSetup(drv_accelHandle_t handle,
        uint8_t adress,
        uint8_t data,
        uint8_t value);

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param handle [description]
 * @return [description]
 */
drv_accelData_t drv_accelRead(drv_accelHandle_t handle);

#ifdef	__cplusplus
}
#endif

#endif	/* NRF_MMA8453Q_H */
