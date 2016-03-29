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

//initializes the peripheral of i2c
nrf_drv_twi_t init_drv();

/*Setting up resolution of accelerometer :
inputs:
p_instance -
address - Accelerometer's address - 0x1D
p_data - register's address
length - number of bytes to be sent
xfer_pending - After a specified number of bytes, transmission will be suspended
(if xfer_pending is set) or stopped (if not), just put false
*/
 void setup_accel(nrf_drv_twi_t *p_instance, uint8_t address, uint8_t * p_data,
         uint8_t * Value,
         uint32_t length,
         bool xfer_pending);
/*Reading data xyz
Inputs:
pointer to save x,y,z values
*/
 bool read_xyz(uint32_t *x, uint32_t *y, uint32_t *z);




#ifdef	__cplusplus
}
#endif

#endif	/* NRF_MMA8453Q_H */
