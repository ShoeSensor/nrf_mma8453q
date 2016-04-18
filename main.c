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

#include <stdbool.h>
#include "nrf_delay.h"
#include "nrf_mma8453q.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_uartDriver.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_config.h"

static drv_accelHandle_t accelHandle;
//static drv_accelData_t accelData;
static char uartBuf[100];

static void readHandler(drv_accelData_t accelData)
{
    sprintf(uartBuf, "X: %d Y: %d Z: %d\r\n", accelData.x, accelData.y, accelData.z);
    uart_write(uartBuf, strlen(uartBuf));
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    drv_twiConfig_t twiConf = {
        .sdaPin = TWI0_CONFIG_SDA,
        .sclPin = TWI0_CONFIG_SCL,
        .twiFreq = TWI0_CONFIG_FREQUENCY,
        .id = TWI0_INSTANCE_INDEX,
        .enable = true
    };
    drv_accelConfig_t accelConf = {
        .gRange = FULL_SCALE_RANGE_4g,
        .highRes = false,
        .address = 0x1D,
        .samplingRate = DATA_RATE_800,
        .readHandler = readHandler
    };
    accelHandle = drv_accelInit(&twiConf);
    drv_accelConfigure(accelHandle, &accelConf);
    uart_init();
    while(1) {
        drv_accelRead(accelHandle);
    }
}
