/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : m_app_twi.c
 *
 * Date : 2016/09/22
 *
 * Usage: nRF51 I2C(TWI) read/write function
 *
 ****************************************************************************
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

#ifndef __M_APP_TWI_H__
#define __M_APP_TWI_H__

#include "app_twi.h"

#define MAX_MULTI_DEVICE_NUM 5

void app_twi_schedule_multi_device_multi_read(app_twi_t *p_app_twi,
					      uint8_t ui8MultiDeviceI2cBase[],
					      uint8_t ui8MultiDeviceSlaveAddr[],
					      uint8_t ui8MultiDeviceRegStartAddr[],
					      uint8_t ui8MultiDeviceByteToRead[],
					      uint8_t ui8MultiDeviceNum,
					      uint8_t m_buffer[],
					      app_twi_callback_t cb_fcn,
					      void* p_user_data);

void app_twi_schedule_multi_read(app_twi_t *p_app_twi,
				 uint8_t dev_addr,
				 uint8_t reg_addr,
				 uint8_t m_buffer[],
				 uint8_t data_len,
				 app_twi_callback_t cb_fcn,
				 void* p_user_data);

void app_twi_schedule_multi_write(app_twi_t *p_app_twi,
				  uint8_t dev_addr,
				  uint8_t reg_addr,
				 uint8_t m_buffer[],
				  uint8_t data_len,
				  app_twi_callback_t cb_fcn,
				  void* p_user_data);

ret_code_t app_twi_perform_single_read(app_twi_t *p_app_twi,
				       uint8_t dev_addr,
				       uint8_t reg_addr,
				       uint8_t *p_reg_value);

ret_code_t app_twi_perform_multi_read(app_twi_t *p_app_twi,
				      uint8_t dev_addr,
				      uint8_t reg_addr,
				      uint8_t reg_values[],
				      uint8_t data_len);

ret_code_t app_twi_perform_single_write(app_twi_t *p_app_twi,
					uint8_t dev_addr,
					uint8_t reg_addr,
					uint8_t reg_value);

ret_code_t app_twi_perform_multi_write(app_twi_t *p_app_twi,
				       uint8_t dev_addr,
				       uint8_t reg_addr,
				       uint8_t reg_values[],
				       uint8_t data_len);


#endif //__M_APP_TWI_H__
