/** \file dogm163.c ******************************************************
 *
 *             Project: MAXREFDES70
 *             Filename: dogm163.c
 *         Description: This module contains the display module functions for the
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
#include <stdio.h>

#include "dogm163.h"
#include "batteryLevel.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"


extern void Delay(uint32_t dlyTicks);

/**************************************************************************//**
 * @brief      Transmit data on the SPI interface.
 *
 * @param[in]  data    Pointer to the data to be transmitted.
 * @param[in]  len     Length of data to transmit.
 *
 * @return     EMSTATUS code of the operation.
 *****************************************************************************/
int DOGM163_SpiTransmit(uint8_t* data, uint32_t len)	{

	/* The design by default sample at the trailing edge (CLKPHA = USART_CTRL_CLKPHA);      */
	/* CLKPHA needs to be modified for every DOGM163 transfer because it uses leading edge. */
	USART1 ->CTRL = USART_CTRL_SYNC | USART_CTRL_MSBF;
	// Clear CS pin.
	GPIO_PinOutClear(gpioPortB, 11);

	while (len > 0) {
		// Transmit starts only when the TX buffer is empty.
		USART_Tx(USART1, *(uint8_t*)data);
		len--;
		data++;

		// Wait for transfer to finish.
		// If USART1 has error, the program is dead!	// to do... exception handling.
		while (!(USART1 ->STATUS & USART_STATUS_TXC))
			;
	}

	// Set CS pin.
	GPIO_PinOutSet(gpioPortB, 11);
	/* The design by default sample at the trailing edge (CLKPHA = USART_CTRL_CLKPHA);      */
	USART1 ->CTRL = USART_CTRL_SYNC | USART_CTRL_MSBF | USART_CTRL_CLKPHA;

	return 0;
}

/**************************************************************************//**
 * @brief   Write a command byte to the DOGM163.
 *
 * @param[in]  byte    Byte to be wirtten to DOGM163.
 *
 * @return     EMSTATUS code of the operation.
 *****************************************************************************/
int DOGM163_WriteCmdByte(uint8_t byte) {
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);

	uint8_t cmd = byte;

	// Clear DOGM163 RS pin (PB13).
	GPIO_PinOutClear(gpioPortB, 13);

	// Write the command byte.
	DOGM163_SpiTransmit(&cmd, 1);

	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	return 0;
}

/**************************************************************************//**
 * @brief   Write a data byte to the DOGM163.
 *
 * @param[in]  byte    Byte to be written to DOGM163.
 *
 * @return  EMSTATUS code of the operation.
 *****************************************************************************/
int DOGM163_WriteDataByte(uint8_t byte) {
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);

	uint8_t data = byte;

	// Set DOGM163 RS pin (PB13).
	GPIO_PinOutSet(gpioPortB, 13);

	// Write the data byte.
	DOGM163_SpiTransmit(&data, 1);

	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	return 0;
}

/**************************************************************************//**
 * @brief   Initialize DOGM163 display.
 *
 * @detail  This function initializes the DOGM163 display module.
 *****************************************************************************/
void DOGM163_Init(void)
{
  // Switch on the power for DOGM163 (PD6).
  GPIO_PinOutSet(gpioPortD, 6);

  // Wait at least 40ms after VDD stable.
  Delay(40);

  /**************************************************************************//**
   * Check the battery voltage level to adjust LCD contrast.
   * Calibrated relationship between LCD contrast and battery voltage level:
   * LCD contrast C5|C4|C3|C2|C1|C0 = 85 - battery voltage level
   * (22: 2.4V ... 58: 3.6V, each level increase = 0.034V).
   *****************************************************************************/
  uint8_t contrast = 85 - batteryLevel(); // 63~27

  // Function Set command: 0x39, 8-bit data, 3-line, instruction table 1.
  DOGM163_WriteCmdByte(0x39);
  Delay(1);

  // Bias Set command: 0x1D, BS 1/4, 3-line.
  DOGM163_WriteCmdByte(0x1D);
  Delay(1);

  // Contrast Set command: 0x70+ C3|C2|C1|C0.
  DOGM163_WriteCmdByte(0x70 + (contrast & 0xF));
  Delay(1);

  // Power/ICON/Contrast Set command: 0x54+ C5|C4, ICON OFF, booster ON, C5|C4.
  DOGM163_WriteCmdByte(0x54 + ((contrast>>4) & 0xF));
  Delay(1);

  // Follower Control command: 0x6C, follower ON, gain Rab[2..0]= [1,0,0].
  DOGM163_WriteCmdByte(0x6C);
  Delay(50);

  // Display ON/OFF command: 0x0F, display ON, cursor Off, cursor position Off. // to do: cursor on or off, position on or off?
  DOGM163_WriteCmdByte(0x0C);
  Delay(1);

  // Clear Display command: 0x01, clear display.
  DOGM163_WriteCmdByte(0x01);
  Delay(2);

  // Entry Mode command: 0x06, auto increment.
  DOGM163_WriteCmdByte(0x06);
  Delay(1);
}

/**************************************************************************//**
 * @brief   Power off DOGM163 display.
 *
 * @detail  This function completely shut down the DOGM163 display module.
 *****************************************************************************/
void DOGM163_PowerOff(void)
{
  // DOGM163 CS pin (PB11).
  GPIO_PinOutClear(gpioPortB, 11);
  // DOGM163 RS pin (PB13).
  GPIO_PinOutClear(gpioPortB, 13);
  // Switch off the power for DOGM163 (PD6).
  GPIO_PinOutClear(gpioPortD, 6);

}


int DOGM163_ClearChars(uint8_t start_row, uint8_t start_column, uint8_t number)
{
  for(uint8_t loop = 0; loop < number; loop++)
  {
    DOGM163_WriteCmdByte(0x80 + 0x10*start_row + start_column + loop);  // Set DDRAM Address.
    //Delay(1);
    DOGM163_WriteDataByte(0x20);                                        // Write SPACE to RAM.
    //Delay(1);
  }

  return 0;

}
// If the number is larger than the string length, print the complete string.
// If the number is less than the string length, print the number of chars.
int DOGM163_PrintChars(uint8_t start_row, uint8_t start_column, uint8_t number, char* string)
{
  uint8_t nOfChars = (number > strlen(string)) ? strlen(string): number;

  for(uint8_t loop = 0; loop < nOfChars; loop++)
  {
    DOGM163_WriteCmdByte(0x80 + 0x10*start_row + start_column + loop);  // Set DDRAM Address.
    //Delay(1);
    DOGM163_WriteDataByte(*(string + loop));                            // Write SPACE to RAM.
    //Delay(1);
  }

  return 0;

}

// limit the positive integer to 16 bits (5 decimal digits).
// fixed width, zero padded.
int DOGM163_PrintInteger(uint8_t start_row, uint8_t start_column, uint8_t width, uint16_t integer)
{
  DOGM163_ClearChars(start_row, start_column, width);

  char string[6] = "";
  snprintf(string, width+1, "%0*d", width, integer);

  uint8_t len = strlen(string);

  // Print len charaters.
  for(uint8_t loop = 0; loop < len; loop++)
  {
    DOGM163_WriteCmdByte(0x80 + 0x10*start_row + start_column + loop);  // Set DDRAM Address.
    Delay(1);
    DOGM163_WriteDataByte(*(string + loop));                            // Write DATA to RAM.
    Delay(1);
  }

  return 0;
}

// Print welcome message.
void DOGM163_PrintWelcomeMsg(void)
{
	DOGM163_ClearChars(0, 0, 48);
	DOGM163_PrintChars(0, 0, 16, "Flow/Heat Meter");
	DOGM163_PrintChars(1, 0, 16, "Reference Design");
	DOGM163_PrintChars(2, 0, 16, "MAXIM INTEGRATED");
}

