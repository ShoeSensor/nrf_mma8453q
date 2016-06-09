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
#include "nrf_drv_config.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_error.h"

#define CONVERT_PRIORITY(id)    CONCAT_3(TWI, id, _CONFIG_IRQ_PRIORITY)

static uint8_t accelBuf[6];
static drv_accelHandle_t gpioHandle = NULL;

static nrf_drv_twi_t twiInstances[TWI_COUNT] = {
        NRF_DRV_TWI_INSTANCE(0)
};

static nrf_drv_twi_config_t twiConfigs[TWI_COUNT] = {
        NRF_DRV_TWI_DEFAULT_CONFIG(0)
};

struct drv_accelHandle {
    nrf_drv_twi_t instance;
    uint8_t address;
    bool highRes;
    nrf_drv_twi_config_t conf;
    drv_accelReadHander_t readHandler;
};

static void twiEventHandler(const nrf_drv_twi_evt_t *event, void *context)
{
    drv_accelHandle_t handle = (drv_accelHandle_t)context;
    drv_accelData_t accelData;
    if(event->type == NRF_DRV_TWI_EVT_DONE) {
        if(event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX) {
            if(handle->highRes) {
                accelData.x = (event->xfer_desc.p_secondary_buf[0] << 4 |
                        (event->xfer_desc.p_secondary_buf[1] >> 4 & 3));
                accelData.y = (event->xfer_desc.p_secondary_buf[2] << 4 |
                        (event->xfer_desc.p_secondary_buf[3] >> 4 & 3));
                accelData.z = (event->xfer_desc.p_secondary_buf[4] << 4 |
                        (event->xfer_desc.p_secondary_buf[5] >> 4 & 3));
            } else {
                accelData.x = event->xfer_desc.p_secondary_buf[0];
                accelData.y = event->xfer_desc.p_secondary_buf[1];
                accelData.z = event->xfer_desc.p_secondary_buf[2];
                if(handle->readHandler)
                    handle->readHandler(accelData);
            }
        }
    }
}

static void dataReadyHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    drv_accelRead(gpioHandle);
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
            &response[1], 1);
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
            &response[1], 1);
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

static void accelReInit(drv_accelHandle_t handle)
{
    nrf_drv_twi_uninit(&handle->instance);
    nrf_drv_twi_init(&handle->instance, &handle->conf, twiEventHandler, handle);
    nrf_drv_twi_enable(&handle->instance);
}

static uint32_t gpioteInit()
{
    uint32_t errCode;
    errCode = nrf_drv_gpiote_init();
    if(errCode != NRF_SUCCESS)
        return errCode;

    nrf_drv_gpiote_in_config_t inConf = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);\

    errCode = nrf_drv_gpiote_in_init(DRDY_PIN, &inConf, dataReadyHandler);
    if(errCode != NRF_SUCCESS)
        return errCode;
    nrf_drv_gpiote_in_event_enable(DRDY_PIN, true);
    return NRF_SUCCESS;
}

void drv_accelEnable(drv_accelHandle_t handle)
{
    nrf_drv_twi_enable(&handle->instance);
}

drv_accelHandle_t drv_accelNew(drv_accelConfig_t *conf,
        drv_accelReadHander_t readHandler)
{
    uint8_t response;
    uint32_t errCode;
    drv_accelHandle_t handle = calloc(1, sizeof(nrf_drv_twi_t));
    handle->address = conf->address;
    handle->highRes = conf->highRes;
    handle->readHandler = readHandler;
    nrf_drv_twi_config_t twiConf = twiConfigs[conf->id];
    handle->conf = twiConf;
    handle->instance = twiInstances[conf->id];
    nrf_drv_twi_init(&handle->instance, &twiConf, twiEventHandler, NULL);
    nrf_drv_twi_enable(&handle->instance);
    errCode = setStandby(handle);
    if(errCode != NRF_SUCCESS)
        return NULL;
    // Range
    errCode = setReg(handle, (uint8_t[]){REG_XYZ_DATA_CFG, conf->gRange});
    if(errCode != NRF_SUCCESS)
        return NULL;
    if(conf->intEnable) {
        errCode = setReg(handle, (uint8_t[]){REG_CTRL_REG4, INT_EN_DRDY});
        if(errCode != NRF_SUCCESS)
            return NULL;
        if(gpioteInit() != NRF_SUCCESS)
            return NULL;
        gpioHandle = handle;
    }
    // Resolution
    errCode = nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_CTRL_REG1}, 1, true);
    if(errCode != NRF_SUCCESS)
        return NULL;
    if(conf->highRes) {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response &~RES_MASK});
    } else {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response | RES_MASK});
    }
    if(errCode != NRF_SUCCESS)
        return NULL;
    errCode = nrf_drv_twi_rx(&handle->instance,
            handle->address, &response, 1);
    if(errCode != NRF_SUCCESS)
        return NULL;
    errCode = setReg(handle, (uint8_t[]){REG_CTRL_REG1,
            (response & DATA_RATE_MASK) | (conf->samplingRate << 3)});
    if(errCode != NRF_SUCCESS)
        return NULL;

    accelReInit(handle); // Set non blocking mode
    setActive(handle);
    return handle;
}

uint32_t drv_accelRead(drv_accelHandle_t handle)
{
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_TXRX(handle->address,
            (uint8_t[]){REG_OUT_X_MSB}, 1,
            accelBuf, 3);
    return nrf_drv_twi_xfer(&handle->instance, &desc, NRF_DRV_TWI_FLAG_TX_NO_STOP);
}

void drv_accelDisable(drv_accelHandle_t handle)
{
    nrf_drv_twi_disable(&handle->instance);
}

void drv_accelDelete(drv_accelHandle_t handle)
{
    nrf_drv_twi_uninit(&handle->instance);
    free(handle);
}
