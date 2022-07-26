/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmp3.h"
#include "nrf_drv_twi.h"
#include "common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID  0xD3


/* Variable to store the device address */
static uint8_t dev_addr;

// i2c instance
static const nrf_drv_twi_t i2c_instance = NRF_DRV_TWI_INSTANCE(0);

/*
  i2c read function
*/
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr; 
    ret_code_t ret_code;
    ret_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, &reg_addr, 1, true);
    if(ret_code != NRF_SUCCESS)
    {
        return ret_code;
    }

    ret_code = nrf_drv_twi_rx(&i2c_instance, dev_addr, reg_data, len);
    if (ret_code == NRF_SUCCESS) 
        return BMP3_INTF_RET_SUCCESS;
    else 
        return 1;
}

/*
  i2c write function
*/
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    ret_code_t ret_code;

    uint8_t buff[16];
    
    // Send reg_addr, followed by red_data
    for (int i = 0; i < len + 1; i++) {
        if (i == 0) {
            buff[i] = reg_addr;
        } else {
            buff[i] = reg_data[i - 1];
        }
    }

    ret_code = nrf_drv_twi_tx(&i2c_instance, dev_addr, buff, len + 1, false); 
    if (ret_code == NRF_SUCCESS) 
        return BMP3_INTF_RET_SUCCESS;
    else 
        return 1;
}

/*!
 * Delay function map to COINES platform
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    nrf_delay_us(period);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            NRF_LOG_INFO("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            NRF_LOG_INFO("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            NRF_LOG_INFO("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            NRF_LOG_INFO("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            NRF_LOG_INFO("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            NRF_LOG_INFO("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            NRF_LOG_INFO("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            NRF_LOG_INFO("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/*
  bmp interface selection (i2c)
*/
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf){

    int8_t rslt = BMP3_OK;

    if(bmp3 != NULL){

        /* Bus configuration : I2C */
        if (intf == BMP3_I2C_INTF){

         NRF_LOG_INFO("I2C Interface\n");
         dev_addr = BMP3_ADDR_I2C_PRIM;
         bmp3->read = bmp3_i2c_read;
         bmp3->write = bmp3_i2c_write;
         bmp3->intf = BMP3_I2C_INTF;
        }

        bmp3->delay_us = bmp3_delay_us;
        bmp3->intf_ptr = &dev_addr;
    }

    else{rslt = BMP3_E_NULL_PTR;}

    return rslt;
}
