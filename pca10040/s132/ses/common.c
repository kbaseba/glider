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
//#define BMP3_SHUTTLE_ID  0x50
#define BMP3_SHUTTLE_ID  0xD3


/* Variable to store the device address */
static uint8_t dev_addr;


/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);




//BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t subaddress, uint8_t *pBuffer, uint32_t ReadNumbr, void *intf_ptr)
//{
//      uint8_t dev_addr = *(uint8_t*)intf_ptr;
//      //uint16_t DevAddress = dev_addr << 1;

//      // send register address
//      //HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &subaddress, 1, BUS_TIMEOUT);
//      return nrf_drv_twi_rx(&m_twi, dev_addr, pBuffer, ReadNumbr);
//}



BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr; 
    ret_code_t ret_code;
    ret_code = nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, false);
    if(ret_code != NRF_SUCCESS)
    {
        return ret_code;
    }

    ret_code = nrf_drv_twi_rx(&m_twi, dev_addr, reg_data, len);
    if (ret_code == NRF_SUCCESS) 
        return BMP3_INTF_RET_SUCCESS;
    else 
        return 1;
}




BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    ret_code_t ret_code;
    ret_code = nrf_drv_twi_tx(&m_twi, dev_addr, reg_data, len, false); 
    if (ret_code == NRF_SUCCESS) 
        return BMP3_INTF_RET_SUCCESS;
    else 
        return 1;
    



    //return nrf_drv_twi_tx(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len, false);

    //return nrf_drv_twi_tx (nrf_drv_twi_t const *p_instance, dev_addr, uint8_t const *p_data, (uint8_t)len, bool no_stop)
}


/*!
 * I2C read function map to COINES platform
 */
//BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
//{
//    //uint8_t dev_addr = *(uint8_t*)intf_ptr;

//    uint8_t dev_addr = *(uint8_t*)intf_ptr;
//    uint16_t DevAddress = dev_addr << 1;

//    return nrf_drv_twi_rx (&m_twi, DevAddress, reg_data, sizeof(reg_data));

//    //return nrf_drv_twi_rx (dev_addr, reg_addr, reg_data, sizeof(reg_data));

//    //return nrf_drv_twi_rx (nrf_drv_twi_t const *p_instance, dev_addr, reg_data, (uint8_t)length)
//}


/*!
 * I2C write function map to COINES platform
 */
//BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
//{
//    uint8_t dev_addr = *(uint8_t*)intf_ptr;

//    return nrf_drv_twi_tx(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len, false);

//    //return nrf_drv_twi_tx (nrf_drv_twi_t const *p_instance, dev_addr, uint8_t const *p_data, (uint8_t)len, bool no_stop)
//}

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


BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf){

    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    //****************************

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



//BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
//{
//    ret_code_t err_code;

//    const nrf_drv_twi_config_t twi_config = {
//       .scl                = 27,
//       .sda                = 26,
//       .frequency          = NRF_DRV_TWI_FREQ_100K,
//       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//       .clear_bus_init     = false
//    };
    
//    /* Bus configuration : I2C */
//    if (intf == BMP3_I2C_INTF)
//    {
//      NRF_LOG_INFO("I2C Interface\n");
//      dev_addr = BMP3_ADDR_I2C_PRIM;
//      bmp3->read = bmp3_i2c_read;
//      bmp3->write = bmp3_i2c_write;
//      bmp3->intf = BMP3_I2C_INTF;
//    }

//    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
//    APP_ERROR_CHECK(err_code);

//    nrf_drv_twi_enable(&m_twi);
//}

//BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
//{
//    int8_t rslt = BMP3_OK;
//    struct coines_board_info board_info;

//    if (bmp3 != NULL)
//    {
//        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);
//        if (result < COINES_SUCCESS)
//        {
//            NRF_LOG_INFO(
//                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
//                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
//            exit(result);
//        }

//        result = coines_get_board_info(&board_info);

//#if defined(PC)
//        setbuf(stdout, NULL);
//#endif

//        if (result == COINES_SUCCESS)
//        {
//            if ((board_info.shuttle_id != BMP3_SHUTTLE_ID))
//            {
//                NRF_LOG_INFO("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
//                exit(COINES_E_FAILURE);
//            }
//        }

//        coines_set_shuttleboard_vdd_vddio_config(0, 0);
//        nrf_delay_ms(1000);

//        /* Bus configuration : I2C */
//        if (intf == BMP3_I2C_INTF)
//        {
//            NRF_LOG_INFO("I2C Interface\n");
//            dev_addr = BMP3_ADDR_I2C_PRIM;
//            bmp3->read = bmp3_i2c_read;
//            bmp3->write = bmp3_i2c_write;
//            bmp3->intf = BMP3_I2C_INTF;
//            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
//        }
//        /* Bus configuration : SPI */
//        else if (intf == BMP3_SPI_INTF)
//        {
//            NRF_LOG_INFO("SPI Interface\n");
//            dev_addr = COINES_SHUTTLE_PIN_7;
//            bmp3->read = bmp3_spi_read;
//            bmp3->write = bmp3_spi_write;
//            bmp3->intf = BMP3_SPI_INTF;
//            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
//        }

//        nrf_delay_ms(1000);

//        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

//        nrf_delay_ms(1000);

//        bmp3->delay_us = bmp3_delay_us;
//        bmp3->intf_ptr = &dev_addr;
//    }
//    else
//    {
//        rslt = BMP3_E_NULL_PTR;
//    }

//    return rslt;
//}

//void bmp3_coines_deinit(void)
//{
//    fflush(stdout);

//    coines_set_shuttleboard_vdd_vddio_config(0, 0);
//    nrf_delay_ms(1000);

//    /* Coines interface reset */
//    coines_soft_reset();
//    nrf_delay_ms(1000);
//    coines_close_comm_intf(COINES_COMM_INTF_USB);
//}
