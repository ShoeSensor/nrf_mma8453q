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

#include "nrf_mma8453q.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_error.h"

#define CONVERT_PRIORITY(id)    CONCAT_3(TWI, id, _CONFIG_IRQ_PRIORITY)

struct drv_accelHandle {
    nrf_drv_twi_t instance;
    uint8_t address;
};

const static uint8_t accelRegVal = 1;

drv_accelHandle_t drv_accelInit(drv_accelConfig_t *conf)
{
    drv_accelHandle_t handle = calloc(1, sizeof(nrf_drv_twi_t));
    nrf_drv_twi_config_t twiConf = {
        .scl = conf->sclPin,
        .sda = conf->sdaPin,
        .frequency = conf->twiFreq,
        .interrupt_priority = CONVERT_PRIORITY(0) // Fixme
    };
    handle->instance = (nrf_drv_twi_t )NRF_DRV_TWI_INSTANCE(0); //Fixme
    nrf_drv_twi_init(&handle->instance, &twiConf, NULL, NULL);
    if(conf->enable)
        nrf_drv_twi_enable(&handle->instance);
    return handle;
}

void drv_accelEnable(drv_accelHandle_t handle)
{
    nrf_drv_twi_enable(&handle->instance);
}

bool drv_accelSetup(drv_accelHandle_t handle, uint8_t address, uint8_t data,
        uint8_t value)
{
    uint32_t errCode;
    uint8_t accelBuf[] = { data, value };
    handle->address = address;
    errCode = nrf_drv_twi_tx(&handle->instance,
        address,
        accelBuf,
        2,
        false);
    return (errCode == NRF_SUCCESS);
}

drv_accelData_t drv_accelRead(drv_accelHandle_t handle)
{
    drv_accelData_t accelData;
    uint32_t errCode;
    uint8_t accelBuf[7];

    memset(accelBuf, 0, sizeof(accelBuf));
    memset(&accelData, 0, sizeof(accelData));
    accelBuf[0] = accelRegVal;

    errCode = nrf_drv_twi_rx(&handle->instance,
            handle->address,
            accelBuf,
            7,
            false);

    if (errCode != NRF_SUCCESS) {
        accelData.failed = true;
        return accelData;
    }

    accelData.x = (accelBuf[0] << 4 | (accelBuf[1] >> 4 & 0xE));
    accelData.y = (accelBuf[2] << 4 | (accelBuf[3] >> 4 & 0xE));
    accelData.z = (accelBuf[4] << 4 | (accelBuf[5] >> 4 & 0xE));

    return accelData;
}

void drv_accelDisable(drv_accelHandle_t handle)
{
    nrf_drv_twi_disable(&handle->instance);
}

