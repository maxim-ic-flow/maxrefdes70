/** \file dogm163.h ******************************************************
 *
 *             Project: MAXREFDES70
 *             Filename: dogm163.h
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
#include <stdint.h>

int DOGM163_SpiTransmit(uint8_t* data, uint32_t len);
int DOGM163_WriteCmdByte(uint8_t byte);
int DOGM163_WriteDataByte(uint8_t byte);

void DOGM163_Init(void);
void DOGM163_PowerOff(void);

int DOGM163_ClearChars(uint8_t start_row, uint8_t start_column, uint8_t number);
int DOGM163_PrintChars(uint8_t start_row, uint8_t start_column, uint8_t number, char* string);
int DOGM163_PrintInteger(uint8_t start_row, uint8_t start_column, uint8_t width, uint16_t integer);
void DOGM163_PrintWelcomeMsg(void);





