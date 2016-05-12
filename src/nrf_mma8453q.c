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

#define CONVERT_PRIORITY(id)    CONCAT_3(TWI, id, _CONFIG_IRQ_PRIORITY)

static uint8_t accelBuf[6];

struct drv_accelHandle {
    nrf_drv_twi_t instance;
    uint8_t address;
    bool highRes;
    drv_twiConfig_t *conf;
    drv_accelReadHander_t readHandler;
};

static void twiEventHandler(const nrf_drv_twi_evt_t *event, void *context)
{
    drv_accelHandle_t handle = (drv_accelHandle_t)context;
    drv_accelData_t accelData;
    if(event->type == NRF_DRV_TWI_EVT_DONE) {
    	switch(event->xfer_desc.type) {
			case NRF_DRV_TWI_XFER_RX:
				if(handle->highRes) {
					accelData.x = (event->xfer_desc.p_primary_buf[0] << 4 | (event->xfer_desc.p_primary_buf[1] >> 4 & 3));
					accelData.y = (event->xfer_desc.p_primary_buf[2] << 4 | (event->xfer_desc.p_primary_buf[3] >> 4 & 3));
					accelData.z = (event->xfer_desc.p_primary_buf[4] << 4 | (event->xfer_desc.p_primary_buf[5] >> 4 & 3));
				} else {
					accelData.x = event->xfer_desc.p_primary_buf[0];
					accelData.y = event->xfer_desc.p_primary_buf[1];
					accelData.z = event->xfer_desc.p_primary_buf[2];
					if(handle->readHandler)
						handle->readHandler(accelData);
				}
				break;
			case NRF_DRV_TWI_XFER_TX:
				memset(accelBuf, 0, sizeof(accelBuf));
				if(handle->highRes) {
					nrf_drv_twi_rx(&handle->instance, handle->address,
							accelBuf, 6);
				} else {
					nrf_drv_twi_rx(&handle->instance, handle->address,
							accelBuf, 3);
				}
				break;
			default:
				break;
    	}
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
    nrf_drv_twi_config_t twiConf = {
        .scl = handle->conf->sclPin,
        .sda = handle->conf->sdaPin,
        .frequency = handle->conf->twiFreq,
        .interrupt_priority = CONVERT_PRIORITY(0) // Fixme
    };
    nrf_drv_twi_init(&handle->instance, &twiConf, twiEventHandler, handle);
    nrf_drv_twi_enable(&handle->instance);
}

drv_accelHandle_t drv_accelInit(drv_twiConfig_t *conf)
{
    drv_accelHandle_t handle = calloc(1, sizeof(nrf_drv_twi_t));
    handle->conf = conf;
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

bool drv_accelConfigure(drv_accelHandle_t handle, drv_accelConfig_t *conf,
        drv_accelReadHander_t readHandler)
{
    uint8_t response;
    uint32_t errCode;
    handle->address = conf->address;
    handle->highRes = conf->highRes;
    handle->readHandler = readHandler;
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
                handle->address, &response, 1);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response &~RES_MASK});
    } else {
        errCode = nrf_drv_twi_rx(&handle->instance,
                handle->address, &response, 1);
        setReg(handle, (uint8_t[]){REG_CTRL_REG1, response | RES_MASK});
    }
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = nrf_drv_twi_rx(&handle->instance,
            handle->address, &response, 1);
    if(errCode != NRF_SUCCESS)
        return false;
    errCode = setReg(handle, (uint8_t[]){REG_CTRL_REG1,
            (response & DATA_RATE_MASK) | (conf->samplingRate << 3)});
    if(errCode != NRF_SUCCESS)
        return false;

    setActive(handle);
    accelReInit(handle); // Set non blocking mode
    return (NRF_SUCCESS);
}

uint32_t drv_accelRead(drv_accelHandle_t handle)
{
    return nrf_drv_twi_tx(&handle->instance, handle->address,
            (uint8_t[]){REG_OUT_X_MSB}, 1, true);
}

void drv_accelDisable(drv_accelHandle_t handle)
{
    nrf_drv_twi_disable(&handle->instance);
}
