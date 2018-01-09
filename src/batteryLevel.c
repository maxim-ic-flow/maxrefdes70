

/** \file batteryLevel.h ******************************************************
 *
 *             Project: MAXREFDES70
 *             Filename: batteryLevel.c
 *         Description: This module contains the battery level test function for the
 *                      implementation of the example
 *                      program for the MAXREFDES70.
 *
 *    Revision History:
 *\n                    09-26-2014    Rev 01.00    MG    Initial release.
 *
/*
 * Copyright (C) 2012 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated Products
 * shall not be used except as stated in the Maxim Integrated Products
 * Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products retains all ownership rights.
 *
 ***************************************************************************/

# include "batteryLevel.h"

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_system.h"
#include "em_vcmp.h"

/***************************************************************************//**
 * @brief
 *   Wait for comparator propagation delay
 ******************************************************************************/
void WaitForComparatorUpdate()
{
  /* Reenable VCMP to trigger a new warm-up cycle */
  VCMP_Disable();
  VCMP_Enable();

  /* Wait for VCMP warm-up */
  while (!VCMP_Ready()) ;
}

/***************************************************************************//**
 * @brief
 *   Check battery voltage (connected to EFM32 VDDIO).
 *
 * @details
 *   Continuously compare battery voltage against a VCMP trigger level,
 *   if VCMPOUT is negative, use that level as battery voltage.
 ******************************************************************************/
int batteryLevel(void)
{
  /* Declare VCMP Init struct */
  VCMP_Init_TypeDef vcmp =
  {
    false,                              /* Half bias current */
    7,                                  /* Bias current configuration */
    false,                              /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
    vcmpHystNone,                       /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    VCMP_VoltageToLevel(3.3),           /* Trigger level */
    true                                /* Enable VCMP after configuration */
  };

  /* Initialize VCMP */
  CMU_ClockEnable(cmuClock_VCMP, true);
  VCMP_Init(&vcmp);

  /* Check from VDD = 2.4V (TRIGLEVEL = 22), to 3.6V (TRIGLEVEL = 58) */
  int batteryLevel = 22;
  for (batteryLevel = 22; batteryLevel < 58; batteryLevel ++)
  {
    /* Set VCMP trigger and check if VDD < VCMP trigger */
    VCMP_TriggerSet(batteryLevel);
    WaitForComparatorUpdate();
    if(!VCMP_VDDHigher())
    	break;

  }

  VCMP_Disable();
  return batteryLevel;
}
