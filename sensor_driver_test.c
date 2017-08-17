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
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf.h"
#include "bsp.h"
#include "gmp102.h"
#include "app_twi.h"
#include "pSensor_util.h"
#include "iir_filter.h"

#define UART_TX_BUF_SIZE            256                  // UART TX buffer size
#define UART_RX_BUF_SIZE            1                    // UART RX buffer size
#define TIMER_GET_DATA_TICK_MS      10                   // Data rate 100Hz
#define MAX_PENDING_TRANSACTIONS    5                    // TWI (I2C)
#define DELAY_MS(ms)	            nrf_delay_ms(ms)
#define IIR_P_LP_ORDER              (14)                 //Pressure low-pass filter order
#define alpha_P                     (1.0f/IIR_P_LP_ORDER)
#define DOF_P                       (1)
//Select IIR filter order below
#define IIR_ORDER           (1)
//#define IIR_ORDER           (2)
//#define IIR_ORDER           (3)
//#define IIR_ORDER           (4)
//#define IIR_ORDER           (5)

const nrf_drv_timer_t m_timer_get_data = NRF_DRV_TIMER_INSTANCE(1);
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
int8_t s8ReadData = 0;

static void event_handler_timer_get_data(nrf_timer_event_t event_type, void* p_context)
{
   
  switch(event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
      s8ReadData = 1;
      break;
        
    default:
      //Do nothing.
      break;
    }    
}

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

/*
  Configure the timer for periodic data read
*/
void init_timer_get_data(void){

  uint32_t time_ticks;
  uint32_t err_code = NRF_SUCCESS;
  nrf_drv_timer_config_t m_nrf_timer_config = {
    .frequency = TIMER1_CONFIG_FREQUENCY,
    .mode = TIMER1_CONFIG_MODE,
    .bit_width = TIMER1_CONFIG_BIT_WIDTH,
    .interrupt_priority = TIMER1_CONFIG_IRQ_PRIORITY,
    .p_context = NULL  //no user data
  };
    
  err_code = nrf_drv_timer_init(&m_timer_get_data, &m_nrf_timer_config, event_handler_timer_get_data);
  APP_ERROR_CHECK(err_code);
    
  time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer_get_data, TIMER_GET_DATA_TICK_MS);
    
  nrf_drv_timer_extended_compare(&m_timer_get_data,
				 NRF_TIMER_CC_CHANNEL0,
				 time_ticks,
				 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				 true);
    
  nrf_drv_timer_enable(&m_timer_get_data);

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
  //pressure low pass filter
  float fP_Pa_lp;
  iirFlt_t histX_P[IIR_ORDER*DOF_P];
  iirFlt_t histY_P[IIR_ORDER*DOF_P];
#if IIR_ORDER == 1
  iirFlt_t coeffA_P[IIR_ORDER + 1] = {1.0, alpha_P - 1.0};
  iirFlt_t coeffB_P[IIR_ORDER + 1] = {alpha_P, 0.0};
#elif IIR_ORDER == 2
  iirFlt_t coeffA_P[IIR_ORDER + 1] = {1., -1.89550185, 0.90069709};
  iirFlt_t coeffB_P[IIR_ORDER + 1] = {0.00129881, 0.00259762, 0.00129881};
#elif IIR_ORDER == 3
  iirFlt_t coeffA_P[IIR_ORDER + 1] = {1., -2.85212611, 2.71498408, -0.86248184};
  iirFlt_t coeffB_P[IIR_ORDER + 1] = {4.70174477e-05, 1.41052343e-04, 1.41052343e-04, 4.70174477e-05};	
#elif IIR_ORDER == 4
  iirFlt_t coeffA_P[IIR_ORDER + 1] = {1., -3.80677094, 5.43876296, -3.45619783, 0.82423302};
  iirFlt_t coeffB_P[IIR_ORDER + 1] = {1.70034519e-06, 6.80138078e-06, 1.02020712e-05, 6.80138078e-06, 1.70034519e-06};
#elif IIR_ORDER == 5
  iirFlt_t coeffA_P[IIR_ORDER + 1] = {1., -4.76069662, 9.07116978, -8.64727116, 4.12391041, -0.78711044};
  iirFlt_t coeffB_P[IIR_ORDER + 1] = {6.14692560e-08, 3.07346280e-07, 6.14692560e-07, 6.14692560e-07, 3.07346280e-07, 6.14692560e-08};	
#endif
  iir_filter_param_t iir_P;	
  iir_P.dof = DOF_P;
  iir_P.order = IIR_ORDER;
  iir_P.histX = histX_P;
  iir_P.histY = histY_P;
  iir_P.coeffA = coeffA_P;
  iir_P.coeffB = coeffB_P;

  //Config and initialize LFCLK
  init_lfclk();

  //Config. and initialize UART
  init_uart();

  //Config. and initialize the periodic data read timer
  init_timer_get_data();

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

  /* GMP102 set P OSR to 4096 */
  s8Res = gmp102_set_P_OSR(GMP102_P_OSR_4096);

  /* GMP102 set T OSR to 2048 */
  s8Res = gmp102_set_T_OSR(GMP102_T_OSR_2048);

  /* First call without wait for P DRDY */
  s8Res = gmp102_measure_P_T(&s32P, &s16T, 0);

  /* set sea leve reference pressure */
  //If not set, use default 101325 Pa for pressure altitude calculation
  set_sea_level_pressure_base(100450.f);


  //Initialize the Pressure low-pass filter
  iirFilterInit(&iir_P);

  for(;;){

    if(s8ReadData){
      
      s8ReadData = 0;

      /* Measure P & T*/
      s8Res = gmp102_measure_P_T(&s32P, &s16T, 1); //Subsequent call with wait for P DRDY
      printf("P(code)=%d\r", s32P);		
      printf("T(code)=%d\r", s16T);
		
      /* Compensation */
      gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
      printf("P(Pa)=%d\r", (s32)fP_Pa);
      printf("100*T(C)=%d\r", (s32)(fT_Celsius*100));

      /* Low-pass filtering pressure */
      filterData((float *)&fP_Pa, (float *)&fP_Pa_lp, &iir_P);
      printf("P_lp(Pa)=%d\r", (s32)fP_Pa_lp);

      /* Pressure Altitude */
      fAlt_m = pressure2Alt(fP_Pa_lp);
      printf("Alt(cm)=%d\r", (s32)(fAlt_m*100));

      printf("\n");

    }
    else{

      __WFI();

    }
  }
}
