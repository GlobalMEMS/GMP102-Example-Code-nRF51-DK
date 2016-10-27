/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : pSensor_util.c
 *
 * Date : 2016/10/26
 *
 * Usage: Pressure Sensor Utility Function
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

/*! @file pSensor_util.c
 *  @brief  Pressure sensor utility function 
 *  @author Joseph FC Tseng
 */

#include <math.h>

static float fp_base_sea_level_Pa = 101325.f;

/*!
 * @brief Set sea level reference pressure base
 *        If not set, the default value is 101325 Pa
 *
 * @param fp_Pa Sea level pressure in Pa
 * 
 * @return None
 *
 */
void set_sea_level_pressure_base(float fp_Pa){

  fp_base_sea_level_Pa = fp_Pa;

}

/*!
 * @brief Pressure altitude conversion
 *        See https://en.wikipedia.org/wiki/Pressure_altitude
 *
 * @param fp_Pa Calibrated pressure in Pa
 * 
 * @return Altitude in m
 *
 */
float pressure2Alt(float fp_Pa){

  return 44307.694f * (1.0f - pow((fp_Pa / fp_base_sea_level_Pa), 0.190284f));

}
