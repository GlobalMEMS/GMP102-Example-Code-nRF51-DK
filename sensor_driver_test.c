/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Date : 2016/09/22
 *
 * Usage: GMP102 Sensor Driver Test for nRF51-DK
 *
 ****************************************************************************
 * @section License
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
 
/*! @file sensor_driver_test.c
 *  @brief  GMP102 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf.h"
#include "bsp.h"
#include "gmp102.h"
#include "app_twi.h"
#include "pSensor_util.h"

#define UART_TX_BUF_SIZE            256                  // UART TX buffer size
#define UART_RX_BUF_SIZE            1                    // UART RX buffer size
#define MAX_PENDING_TRANSACTIONS    5                    // TWI (I2C)
#define DELAY_MS(ms)	            nrf_delay_ms(ms)

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static void event_handler_uart(app_uart_evt_t * p_event){

  uint8_t cr;

  switch (p_event->evt_type){

  case APP_UART_DATA_READY: //echo

    while(app_uart_get(&cr) == NRF_SUCCESS){
      printf("%c", cr);
    }
    break;
  case APP_UART_TX_EMPTY:
    //do nothin
    break;
  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;
  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

void init_lfclk(void){

  uint32_t err_code;

  // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
  // buttons with the use of APP_TIMER

  err_code = nrf_drv_clock_init(NULL);
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request();

}

void init_uart(void)
{
  uint32_t err_code;

  app_uart_comm_params_t const comm_params =
    {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
      APP_UART_FLOW_CONTROL_DISABLED,
      false,
      UART_BAUDRATE_BAUDRATE_Baud115200
    };

  APP_UART_FIFO_INIT(&comm_params,
		     UART_RX_BUF_SIZE,
		     UART_TX_BUF_SIZE,
		     event_handler_uart,
		     APP_IRQ_PRIORITY_LOW,
		     err_code);

  APP_ERROR_CHECK(err_code);
}

/**
 * Initialize two wire interface (I2C)
 */
void init_twi(nrf_twi_frequency_t clk){

  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = clk,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };

  APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
  APP_ERROR_CHECK(err_code);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  s8 s8Res; 
  bus_support_t gmp102_bus;
  float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fAlt_m;
  s16 s16T;
  s32 s32P;

  //Config and initialize LFCLK
  init_lfclk();

  //Config. and initialize UART
  init_uart();

  //Config. and initialize TWI (I2C)
  init_twi(NRF_TWI_FREQ_400K);
	
  /* GMP102 I2C bus setup */
  bus_init_I2C(&gmp102_bus, &m_app_twi, GMP102_7BIT_I2C_ADDR);  //Initialize I2C bus
  gmp102_bus_init(&gmp102_bus); //Initailze GMP102 bus to I2C

  /* GMP102 soft reset */
  s8Res = gmp102_soft_reset();
	
  /* Wait 100ms for reset complete */
  DELAY_MS(100);
	
  /* GMP102 get the pressure calibration parameters */
  s8Res = gmp102_get_calibration_param(fCalibParam);
	
  /* GMP102 initialization setup */
  s8Res = gmp102_initialization();

  /* GMP102 set P OSR to 1024 */
  s8Res = gmp102_set_P_OSR(GMP102_P_OSR_1024);

  /* set sea leve reference pressure */
  //If not set, use default 101325 Pa for pressure altitude calculation
  set_sea_level_pressure_base(100450.f);

  for(;;)
    {
      /* Measure P */
      s8Res = gmp102_measure_P(&s32P);
      printf("P(code)=%d\r", s32P);
		
      /* Mesaure T */
      s8Res = gmp102_measure_T(&s16T);
      printf("T(code)=%d\r", s16T);
		
      /* Compensation */
      gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
      printf("P(Pa)=%d\r", (s32)fP_Pa);
      printf("100*T(C)=%d\r", (s32)(fT_Celsius*100));

      /* Pressure Altitude */
      fAlt_m = pressure2Alt(fP_Pa);
      printf("Alt(cm)=%d\r", (s32)(fAlt_m*100));

      printf("\n");
      /* Delay 1 sec */
      DELAY_MS(1000);
    }
}
