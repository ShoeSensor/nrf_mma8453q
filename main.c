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
#include "nrf_mma8453q.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"

#include "nrf_delay.h"

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    nrf_drv_twi_t defaultConf1=  init_drv();
    uint32_t x,y,z;
    uint8_t data = 0xe;
    uint8_t value = 0x1;
    setup_accel(&defaultConf1, 0x1D, &data, &value, 1, NULL);
    while (true)
    {
        read_xyz(x,y,z);

    }
}
