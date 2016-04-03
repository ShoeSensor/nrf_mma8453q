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
    bool highRes;
};

const static uint8_t accelRegVal = 1;


static void setStandby(drv_accelHandle_t handle)
{
    uint8_t response[2] = {REG_CTRL_REG1, 0};;
    nrf_drv_twi_rx(&handle->instance, handle->address, &response[1], 1, true);
    response[1] &= ~ACTIVE_MASK;
    nrf_drv_twi_tx(&handle->instance, handle->address, response, 2, true);
}

static void setActive(drv_accelHandle_t handle)
{
    uint8_t response[2] = {REG_CTRL_REG1, 0};
    nrf_drv_twi_rx(&handle->instance, handle->address, &response[1], 1, true);
    response[1] |= ACTIVE_MASK;
    nrf_drv_twi_tx(&handle->instance, handle->address, response, 2, true);
}

uint32_t setReg(drv_accelHandle_t handle, uint8_t data[2])
{
    return nrf_drv_twi_tx(&handle->instance, handle->address,
        data, 2, true);
}

drv_accelHandle_t drv_accelInit(drv_twiConfig_t *conf)
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

bool drv_accelConfigure(drv_accelHandle_t handle, drv_accelConfig_t *conf)
{
    uint8_t response;
    uint32_t errCode;
    handle->address = conf->address;
    setStandby(handle);
    // Range
    errCode = setReg(handle, (uint8_t[]){REG_XYZ_DATA_CFG, conf->gRange});
    if(errCode != NRF_SUCCESS)
        return false;
    // Resolution
    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_CTRL_REG1}, 1, false);
    if(errCode != NRF_SUCCESS)
        return false;
    if(conf->highRes) {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1, true);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response &~RES_MASK});
    } else {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1, true);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response | RES_MASK});
    }
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = nrf_drv_twi_rx(&handle->instance,
            handle->address, &response, 1, true);\
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = setReg(handle, (uint8_t[]){REG_CTRL_REG1,
            (response & DATA_RATE_MASK) | (conf->samplingRate << 3)});
    if(errCode != NRF_SUCCESS)
        return false;
    setActive(handle);
    return (NRF_SUCCESS);
}

drv_accelData_t drv_accelRead(drv_accelHandle_t handle)
{
    drv_accelData_t accelData;
    uint32_t errCode;
    uint8_t accelBuf[6];

    memset(accelBuf, 0, sizeof(accelBuf));

    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_CTRL_REG1}, 1, false);

    if (errCode != NRF_SUCCESS)
        goto err;

    if(handle->highRes) {
        errCode = nrf_drv_twi_rx(&handle->instance, handle->address,
                accelBuf, 6, true);
        accelData.x = (accelBuf[0] << 4 | (accelBuf[1] >> 4 & 0xE));
        accelData.y = (accelBuf[2] << 4 | (accelBuf[3] >> 4 & 0xE));
        accelData.z = (accelBuf[4] << 4 | (accelBuf[5] >> 4 & 0xE));
    } else {
        errCode = nrf_drv_twi_rx(&handle->instance, handle->address,
                accelBuf, 3, true);
        accelData.x = (accelBuf[0] << 4);
        accelData.y = (accelBuf[1] << 4);
        accelData.z = (accelBuf[2] << 4);
    }

    if (errCode != NRF_SUCCESS)
        goto err;

    accelData.x = (accelData.x > 2047) ? accelData.x - 4096 : accelData.x;
    accelData.y = (accelData.y > 2047) ? accelData.y - 4096 : accelData.y;
    accelData.z = (accelData.z > 2047) ? accelData.z - 4096 : accelData.z;

    return accelData;

    err:
        accelData.failed = true;
        return accelData;
}

void drv_accelDisable(drv_accelHandle_t handle)
{
    nrf_drv_twi_disable(&handle->instance);
}

