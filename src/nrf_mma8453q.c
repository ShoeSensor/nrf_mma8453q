/*
 * Copyright 2016 Bart Monhemius.
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

#define DRV_TWI_CONF_DEFAULT (nrf_drv_twi_config_t) {   \
        .sda = TWI0_CONFIG_SDA,                         \
        .scl = TWI0_CONFIG_SCL,                         \
        .frequency = TWI0_CONFIG_FREQUENCY,             \
        .interrupt_priority = TWI0_CONFIG_IRQ_PRIORITY}

static uint8_t accelBuf[6];
static bool isTwiInit = false;

struct drv_accelHandle {
    nrf_drv_twi_t instance;
    uint8_t address;
    bool highRes;
    drv_accelReadHander_t readHandler;
    bool isStopped;
};

static void twiEventHandler(const nrf_drv_twi_evt_t *event, void *context)
{
    drv_accelHandle_t handle = (drv_accelHandle_t)context;
    drv_accelData_t accelData;
    switch(event->type) {
        case NRF_DRV_TWI_RX_DONE:
            if(handle->highRes) {
                accelData.x = (event->p_data[0] << 4 | (event->p_data[1] >> 4 & 3));
                accelData.y = (event->p_data[2] << 4 | (event->p_data[3] >> 4 & 3));
                accelData.z = (event->p_data[4] << 4 | (event->p_data[5] >> 4 & 3));
            } else {
                accelData.x = event->p_data[0];
                accelData.y = event->p_data[1];
                accelData.z = event->p_data[2];
                if(handle->readHandler)
                    handle->readHandler(accelData);
            }
            if(!handle->isStopped)
                nrf_drv_twi_tx(&handle->instance, handle->address,
                    (uint8_t[]){REG_OUT_X_MSB}, 1, true);
            break;
        case NRF_DRV_TWI_TX_DONE:
            if(handle->highRes) {
                nrf_drv_twi_rx(&handle->instance, handle->address,
                        accelBuf, 6, false);
            } else {
                nrf_drv_twi_rx(&handle->instance, handle->address,
                        accelBuf, 3, false);
            }
            break;
        default:
            break;
    }
}

static uint32_t setStandby(drv_accelHandle_t handle)
{
    uint32_t errCode;
    uint8_t response[2] = {REG_CTRL_REG1, 0};

    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
                response, 1, true);
    if(errCode != NRF_SUCCESS)
        return errCode;
    errCode = nrf_drv_twi_rx(&handle->instance, handle->address,
            &response[1], 1, false);
    if(errCode != NRF_SUCCESS)
        return errCode;
    response[1] &= ~ACTIVE_MASK;
    return nrf_drv_twi_tx(&handle->instance, handle->address,
            response, 2, false);
}

static uint32_t setActive(drv_accelHandle_t handle)
{
    uint32_t errCode;
    uint8_t response[2] = {REG_CTRL_REG1, 0};
    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
                response, 1, false);
    if(errCode != NRF_SUCCESS)
        return errCode;
    errCode = nrf_drv_twi_rx(&handle->instance, handle->address,
            &response[1], 1, false);
    if(errCode != NRF_SUCCESS)
        return errCode;
    response[1] |= ACTIVE_MASK;
    return nrf_drv_twi_tx(&handle->instance, handle->address,
            response, 2, true);
}

static uint32_t setReg(drv_accelHandle_t handle, uint8_t data[2])
{
    return nrf_drv_twi_tx(&handle->instance, handle->address,
        data, 2, false);
}

drv_accelHandle_t drv_accelNew(drv_accelConfig_t *conf,
        drv_accelReadHander_t readHandler)
{
    uint8_t response;
    uint32_t errCode;
    drv_accelHandle_t handle = calloc(1, sizeof(struct drv_accelHandle));
    handle = &(struct drv_accelHandle) {
            .address = conf->address,
            .highRes = conf->highRes,
            .readHandler = readHandler,
            .instance = (nrf_drv_twi_t )NRF_DRV_TWI_INSTANCE(0)
    };
    nrf_drv_twi_config_t twiConf = DRV_TWI_CONF_DEFAULT;
    if(!isTwiInit) {
        nrf_drv_twi_init(&handle->instance, &twiConf, NULL, NULL);
        nrf_drv_twi_enable(&handle->instance);
    }

    errCode = setStandby(handle);
    if(errCode != NRF_SUCCESS)
        return false;
    // Range
    errCode = setReg(handle, (uint8_t[]){REG_XYZ_DATA_CFG, conf->gRange});
    if(errCode != NRF_SUCCESS)
        return false;
    // Resolution
    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_CTRL_REG1}, 1, true);
    if(errCode != NRF_SUCCESS)
        return false;
    if(conf->highRes) {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1, false);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response &~RES_MASK});
    } else {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1, false);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response | RES_MASK});
    }
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = nrf_drv_twi_rx(&handle->instance,
            handle->address, &response, 1, false);\
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = setReg(handle, (uint8_t[]){REG_CTRL_REG1,
            (response & DATA_RATE_MASK) | (conf->samplingRate << 3)});
    if(errCode != NRF_SUCCESS)
        return false;

    setActive(handle);
    nrf_drv_twi_uninit(&handle->instance);
    nrf_drv_twi_init(&handle->instance, &twiConf, twiEventHandler, handle);
    nrf_drv_twi_enable(&handle->instance);
    isTwiInit = true;
    return (NRF_SUCCESS);
}

void drv_accelEnable(drv_accelHandle_t handle)
{
    nrf_drv_twi_enable(&handle->instance);
    setActive(handle);
}


uint32_t drv_accelStartRead(drv_accelHandle_t handle)
{
    return nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_OUT_X_MSB}, 1, true);
}

void drv_accelStopRead(drv_accelHandle_t handle)
{
    handle->isStopped = true; //FIXME Mutual exclusion needed?
}

void drv_accelDisable(drv_accelHandle_t handle)
{
    setStandby(handle);
    nrf_drv_twi_disable(&handle->instance);
}
