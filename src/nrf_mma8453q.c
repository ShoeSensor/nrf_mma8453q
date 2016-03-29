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

#include "nrf_mma8453q.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"

#include "nrf_delay.h"

nrf_drv_twi_t defaultConf;

//init
nrf_drv_twi_t init_drv()
{
    defaultConf = (nrf_drv_twi_t) NRF_DRV_TWI_INSTANCE(0);
    nrf_drv_twi_init(&defaultConf, NULL, NULL, NULL);
    nrf_drv_twi_enable(&defaultConf);
    return defaultConf;
}

//Setting up resolution of accelerometer to 4g and 10bit
void setup_accel(nrf_drv_twi_t *p_instance,  /**< TWI instance */
        uint8_t address,                     /**< Sensor address */
        uint8_t *p_data,                     /**< Pointer to transmit buffer */
        uint8_t *Value,                      /**< Value of sensitivity */
        uint32_t length,                     /**< Bytes to be sent */
        bool xfer_pending)
{
    uint32_t errCode;
    errCode = nrf_drv_twi_tx(p_instance,
            address,
            p_data,
            length,
            xfer_pending);
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_twi_tx(p_instance,
            address,
            Value,
            length,
            xfer_pending);
    APP_ERROR_CHECK(errCode);
}

bool read_xyz(uint32_t *x, uint32_t *y, uint32_t *z) /**< Returned values */
{
    uint32_t errCode;
    uint8_t buf[6];
    uint8_t address = 0x1d;
    uint8_t reg_pointer = 0x01;

    //send the register
    errCode = nrf_drv_twi_tx(&defaultConf,   /**< TWI instance */
            address                            /**< Sensor address */
            ,&reg_pointer                            /**< Pointer to transmit buffer */
            ,1,                              /**< # of bytes to send */
            NULL
            );
    if(errCode != NRF_SUCCESS)
        return false;

    //store everything in the buffer
    nrf_drv_twi_rx ( &defaultConf,            /**< TWI instance */
            address,                             /**< Sensor address */
            (uint8_t*)&buf,                   /**< Pointer to transmit buffer */
            6,                                /**< # of bytes to be received */
            NULL);
    *x = (buf[0] << 4 | (buf[1] >> 4 & 0xE)); /**< Combining two bytes of 1 axle*/
    *y = (buf[2] << 4 | (buf[3] >> 4 & 0xE));
    *z = (buf[4] << 4 | (buf[5] >> 4 & 0xE));
    if(errCode == NRF_SUCCESS)
        return true;
}

