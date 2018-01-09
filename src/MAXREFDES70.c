/** \file MAXREFDES70.c ******************************************************
 *
 *             Project: MAXREFDES70
 *             Filename: MAXREFDES70.c
 *         Description: This module contains the Main application for the
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

/*!\mainpage Main Page
 *
 * \section intro_sec Introduction
 *
 * This is the code documentation for the MAXREFDES70# reference design.
 * \n
 * \n The Files page contains the File List page and the Globals page.
 * \n
 * \n The Globals page contains the Functions, Variables, and Macros sub-pages.
 * \n
 * \image html MAXREFDES70_Block_Diagram_V10.png "MAXREFDES70# System Block Diagram"
 * \n
 * \image html MAXREFDES70_Firmware_Main_Flowchart_V10.png "MAXREFDES70# Firmware Main Flowchart"
 * \n
 * \n
 * \image html MAXREFDES70_Firmware_ISR_Flowchart_V10.png "MAXREFDES70# Firmware ISR Flowchart"
 *
 */
//Micro Specific Include Files
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

//Standard library call Include Files
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

#include "lookUpTables.h"
#include "dogm163.h"

/*******************************Opcode and Register Definition*********************************/
//MAX35101 Opcode Defines
#define TOF_UP				0x00
#define TOF_DOWN			0x01
#define TOF_DIFF			0x02
#define TEMPERATURE			0x03
#define RESET				0x04
#define INITIALIZE			0x05
#define TRANSFER_TO_FLASH	0x06
#define EVTMG1				0x07
#define EVTMG2				0x08
#define EVTMG3				0x09
#define HALT				0x0a
#define LDO_TIMED			0x0b
#define LDO_ON				0x0c
#define LDO_OFF				0x0d
#define CALIBRATES			0x0e
#define READ_FLASH			0x90
#define WRITE_FLASH			0x10
#define BLOCK_ERASE_FLASH	0x13

//MAX35101 Register Memory Map Base Address
//(Write Opcode Values, Read routines will set the MSBit to get correct read opcode value)
#define SECONDS 			0x30
#define MINS_HRS 			0x31
#define DAY_DATE			0x32
#define MONTH_YEAR			0x33
#define INTERRUPT_STATUS 	0xFE
#define T1_REG				0xE7
#define T2_REG				0xE9
#define T3_REG				0xEB
#define T4_REG				0xED
#define T1_AVG				0xF0
#define T2_AVG				0xF2
#define T3_AVG				0xF4
#define T4_AVG				0xF6
#define HIT1UP_REG			0xC5
#define HIT2UP_REG			0xC7
#define HIT3UP_REG			0xC9
#define HIT4UP_REG			0xCB
#define HIT5UP_REG			0xCD
#define HIT6UP_REG			0xCF
#define AVGUP_REG			0xD1
#define HIT1DOWN_REG		0xD4
#define HIT2DOWN_REG		0xD6
#define HIT3DOWN_REG		0xD8
#define HIT4DOWN_REG		0xDA
#define HIT5DOWN_REG		0xDC
#define HIT6DOWN_REG		0xDE
#define AVGDOWN_REG			0xE0
#define TOF_DIFF_REG		0xE2
#define TOF_DIFF_AVG_REG	0xE5

//MAX35101 Bit weighting definition for interrupt status Register
#define INTERRUPT_REG_TO 			0x8000
#define INTERRUPT_REG_TOF 			0x1000
#define INTERRUPT_REG_TE 			0x0800
#define INTERRUPT_REG_TOF_EVTMG 	0x0200
#define INTERRUPT_REG_TEMP_EVTMG	0x0100
#define INTERRUPT_REG_TOF_FLASH 	0x0080
/*******************************END - Opcode and Register Definition*********************************/


//Use 3 temperature ports, identify the hot inlet as T1, cold as T2, reference as T3 (Reading the ports from MAX35101)
#define NUMBER_TEMPERATUE_PORTS_USED	3
#define HOT_TEMP_PORT 					0
#define COLD_TEMP_PORT					1
#define REFERENCE_TEMP_PORT 			2
/*******************************Data Structure Definitions*********************************/
/*Temperature port, all temperature ports, hit data, flow data, energy data, Point-in-Time data,  */

#define USING_EVENT_TIMING_MODES_READ_AVERAGE

//A Temperature results structure, containing raw data, and actually calculated temp (Relevant if its an inlet of outlet RTD)
typedef struct {
	uint32_t Register_Value;
	float TemperatureDegreeC;
} Temperature_ResultsStruct;

//An array of temperature ports, Inlet RTD, outlet RTD, and reference. All temperature data
typedef struct {
	Temperature_ResultsStruct TempResultsAllPorts[NUMBER_TEMPERATUE_PORTS_USED];
} TemperatureResultsAllPortsStruct;

//Hit data and ratio data for TOF
typedef struct {
	uint32_t Hit1Data;
	uint32_t Hit2Data;
	uint32_t Hit3Data;
	uint32_t Hit4Data;
	uint32_t Hit5Data;
	uint32_t Hit6Data;
	uint32_t HitAverageData;
	char t1Ratiot2;
	char t2RatiotIdeal;
} Hit_ResultsStruct;


typedef struct{
	struct tm Time;
	uint32_t milliSeconds;
}tm_withMilli;

typedef struct {

#ifndef USING_EVENT_TIMING_MODES_READ_AVERAGE
	Hit_ResultsStruct HitDownData;
	Hit_ResultsStruct HitUpData;
#endif
	int32_t TOF_DiffData;
	float TOF_DIFF_DeltaT_S;
	float FlowVelocity_mPerS;
	float VolumetricFlow_m3PerS;
	float VolumetricFlowGainFactor;
	float VolumetricFlowCorrected_m3PerS;
} Flow_ResultsStruct;

typedef struct {
	float MassFlow_kgPerh;
	float EnthalpyAtHot_JperKg;
	float EnthalpyAtCold_JperKg;
	float EnthalpyDelta_JperKg;
} Energy_ResultsStruct;

typedef struct {	//POT is a Point in Time
	Flow_ResultsStruct 					POT_FlowFactors;
	Energy_ResultsStruct				POT_EnergyFactors;
	tm_withMilli						POT_TimeData;
	TemperatureResultsAllPortsStruct 	POT_TemperatureData;
} PointOfTimeSampleDataStruct;



/* Display Mode */
 enum {
	Display_Off = 0,
	Display_Welcome = Display_Off + 1,
	Display_Clock = Display_Welcome + 1,
	Display_Temp = Display_Clock + 1,
	Display_TOFDIFF = Display_Temp + 1,
	Display_Volumetric_Flow = Display_TOFDIFF + 1,
	Display_Total_Volume = Display_Volumetric_Flow + 1,
	Display_Energy = Display_Total_Volume + 1,
	Display_TDF_Config = Display_Energy + 1,
	Display_TDM_Config = Display_TDF_Config + 1,
	Display_TMF_Config = Display_TDM_Config + 1,
	Display_TMM_Config = Display_TMF_Config + 1,
	Display_Last = Display_TMM_Config + 1
} ;
 /******************************* END - Data Structure Definitions*********************************/


 /******************************* Some GLOBAL variables, and Function Definitions*********************************/
 //Global Generic Defines
#define T4MHZ				(float)250/(float)1000000000						//Assume Ideal 250nS t4MHz period
#define LSBIT_TOF_VALUE 	(1/(float)65536) * T4MHZ							//Fractional portion of a TOF measurement is quantized to a 16 bit value of the 4MHz Clock
#ifndef PI
#define PI			3.14  //Make sure this is defines somewhere
#endif
#define SECONDS_PER_HR 	3600

 //Global Variables
TemperatureResultsAllPortsStruct Last_TempUpdate;	//The Most Recent temperature measurement, stored here since temperature updates and tof updates occur at different rates
static volatile uint32_t msTicks; /* counts 1ms timeTicks */

//#define NUMBER_OF_POTS_BEFORE_CALCULATION 4		//Number of data points to collect before doing the piecewise integration, giving the resulting energy consumption
//PointOfTimeSampleDataStruct POT_Data[NUMBER_OF_POTS_BEFORE_CALCULATION + 1];	//Actual array of datapoints
//PointOfTimeSampleDataStruct POT_Data_WrapAround;	//Must keep last point of previous accumulation to use as first point of the next accumulation etc.
PointOfTimeSampleDataStruct POT_Data[1];	//Actual array of datapoints
PointOfTimeSampleDataStruct POT_Data_Last;
int 	POTCount = 0;		//Tracker
float 	TotalEnergy = 0;	//TOTAL ENERGY is STORED HERE	//todo: customer must save/transmit this data at their defined rate, resetting the variable for continual accumulations
float 	TotalVolume_m3 = 0;
int 	POR = 0;			//Flag keeping track of a power on reset
uint16_t TDF, TDM, TMF, TMM;	//current working TDF, TDM, TMF, TMM values.
uint16_t reg = 0;
char output[49];	// Characters to display on LCD screen

uint8_t DisplayMode = Display_Off;			//What mode is the screen displaying
uint8_t previousDisplayMode = Display_Off;	//What mode was the screen displaying previously
int displayRepeat = 0;
#define MAX_DISPLAY_REPEAT 9000			//without pressing SW2, LCD keeps on for 1 hour and then turns off.
bool displayPowerOff = true;
bool EVT_STARTED = false;
uint32_t delayCount = 0;

void Display(void);
void MAX35101_SetTime(void);


//Usable Calculation routines
bool Calcuate_Temperature(uint32_t tempRegisterData, uint32_t RefRegisterData, float* TempDestination);
bool Calculate_TOF_Velocity(int32_t TOF_DiffData, float* FlowVelocity,	float Temperature, float* TOF_DIFF_DeltaT_S);
void Calculate_TimeDifference( tm_withMilli*  t1, tm_withMilli*  t2, float* difference_sec);
bool Calculate_Volumetric_Flow( float Velocity_mPerS, float* VolumetricFlow_m3PerS, float* VolumetricGainfactor, float*VolumetricFlowCorrected_m3PerS);
bool Calculate_Mass_Flow(float VolumetricFlow_m3PerS, float* MassFlow_kgPerHr, float Temperature);
bool Calculate_Enthalpy(float TemperatureAtCold, float TemperatureAtHot, float* EnthalpyAtCold_JperKg, float* EnthalpyAtHot_JperKg, float* DeltaEnthalpy_JperKg);
bool Calculate_Piecewiese_Energy(PointOfTimeSampleDataStruct* POT1, 	PointOfTimeSampleDataStruct* POT2, float *EnergyForTime_J);
bool Calculate_Piecewise_Volume(PointOfTimeSampleDataStruct* POT1, PointOfTimeSampleDataStruct* POT2, float *Volume_m3);

//Usable MAX35101 communication routines
bool MAX35101_Send_Opcode(char opcode);
bool MAX35101_Read_Register(char address, uint16_t* results);
bool MAX35101_Read_2WordValue(char startingAddress, uint32_t* results);
bool MAX35101_Update_TOF_DIFFData(Flow_ResultsStruct* TOF_DIFF_Results);
bool MAX35101_UpdateAndGetTime(tm_withMilli* RTCTimeStamp);
bool MAX35101_Update_TOF_AVG_DIFFData(Flow_ResultsStruct* TOF_DIFF_Results);
bool MAX35101_Write_Register(char address, uint16_t DatatoWrite);
bool MAX35101_Update_TemperatureData(TemperatureResultsAllPortsStruct* TempResultsToUpdate);

//usable generic routines
float LinearInterpolation (float X1Low, float X2High, float Y1Low, float Y2High, float XnewLookup);

// Format a floating number to a string.
// Newlib-nano formatted printing and parsing of floating number requires large code footprint.
// Not enough space with 32k flash available on the device.
int FloatToString (float fNumber, uint8_t precision, char* output);

//Micro specific block communication setup routines
//todo: rewrite the specific functions for the mirco of use
extern void Delay(uint32_t dlyTicks);
static void SPI_setup(uint8_t spiNumber, uint8_t location, bool master);
void init(void);

static void GpioSetup(void);

bool SPI_Send_Byte(char dataByte);
bool SPI_Read_Word(uint16_t* results);
bool MAX35101_SendConfigs(void);	//Generic writing all data registers
#define MAX35101_CS_Low() 	GPIO_PinOutClear(gpioPortB, 8)
#define MAX35101_CS_High()	GPIO_PinOutSet(gpioPortB, 8)

/******************************* END - Some GLOBAL variables, and Function Definitions*********************************/


/******************************* Meter Calculation routines*********************************/
#define RTD_RREF_VALUE 1000
bool Calcuate_Temperature(uint32_t tempRegisterData, uint32_t RefRegisterData, float* TempDestination)
{
	float RTDtoRREF_Ratio, RTDResistanceValue_ohms;
	int XCord, YCord;
	bool success = false;

	RTDtoRREF_Ratio = (float) tempRegisterData / (float) RefRegisterData;//Time for RTD#/RREF
	RTDResistanceValue_ohms = RTDtoRREF_Ratio * RTD_RREF_VALUE;//Ration time RREF = resistance of the RTD#

	//Find the X coordinate in the lookup table by fixing the T to zero: X coordinate is whole degrees C
	for (XCord = 0;	((PT1000_LOOKUPTABLE[XCord][0] < RTDResistanceValue_ohms) && (XCord < 101)); XCord++)	//Start at 0 Degree C, a deference lookup table is resistance of RTD (ohms)
		{;}	//Purely searching for the index
	XCord--; //we stopped when the Resistance of a whole degree was greater than what we wanted, decrement the whole degree and search for the Y
	//Find the Y coordinate in the lookup table by fixing the X to the found value: Y coordinate is 0.1 Degrees C
	for (YCord = 0;	((PT1000_LOOKUPTABLE[XCord][YCord] < RTDResistanceValue_ohms) && (YCord < 10)); YCord++)	//Start at 0 Degree C, a deference lookup table is resistance of RTD (ohms)
		{;}//Purely searching for the index
	YCord--; //we stopped when the Resistance of a 0.1 degree was greater than what we wanted, decrement

	*TempDestination = LinearInterpolation
											(
												PT1000_LOOKUPTABLE[XCord][YCord],		//X1 - //Index was past
												PT1000_LOOKUPTABLE[XCord][YCord + 1],			//X2
												(XCord + (YCord * 0.1)),		//Y1 //index into LUT is degree and .1 degree
												(XCord + (YCord + 1) * 0.1),	//Y2 //Add whole to decimal portion and the interpolated value to the .1 then thats the TEMP
												RTDResistanceValue_ohms	//New X of interest
											);//Returns new Y
	return success;
}


#define PIPELENGTH_IN_FLOW_M	0.0717	//todo: DN20 or DN25 - Specifications - update to spool body
bool Calculate_Flow_Parameters(Flow_ResultsStruct* TOF_DIFF_ResultsToUse, TemperatureResultsAllPortsStruct* TemperatureResultsToUse)
{
	bool success = false;
	Calculate_TOF_Velocity(
			TOF_DIFF_ResultsToUse->TOF_DiffData,
			&TOF_DIFF_ResultsToUse->FlowVelocity_mPerS,
			TemperatureResultsToUse->TempResultsAllPorts[HOT_TEMP_PORT].TemperatureDegreeC,
			&TOF_DIFF_ResultsToUse->TOF_DIFF_DeltaT_S
			);
	Calculate_Volumetric_Flow(TOF_DIFF_ResultsToUse->FlowVelocity_mPerS, &TOF_DIFF_ResultsToUse->VolumetricFlow_m3PerS,  &TOF_DIFF_ResultsToUse->VolumetricFlowGainFactor, &TOF_DIFF_ResultsToUse->VolumetricFlowCorrected_m3PerS);
	return success;
}

bool Calculate_TOF_Velocity(int32_t TOF_DiffData, float* FlowVelocity, float Temperature, float* TOF_DIFF_DeltaT_S)
{
	bool success = false;
	float SpeedOfSound_mperS, VelocityCalcNumerator_m2perS,	VelocityCalcDenominatore_m, Velocity_mPers = 0;
	int TempAtJunncture = (int) Temperature;
	//Current table is mapped with degrees C, just insert whole degree and read out velocity (speed of sound m/S)
	SpeedOfSound_mperS = LinearInterpolation
											(
												TempAtJunncture,		//X1
												TempAtJunncture + 1,	//X2 (increment of 1 is next lookup value +1C)
												(float)SPEED_OF_SOUND_LOOKUP[TempAtJunncture],		//Y1
												(float)SPEED_OF_SOUND_LOOKUP[TempAtJunncture + 1],	//Y2
												Temperature	//New X of interest
											);//Returns new Y
	//Calculate the TOF Differential (Delta t) time in seconds. TODO:Is seconds best?
	*TOF_DIFF_DeltaT_S = (float) TOF_DiffData * LSBIT_TOF_VALUE;
	//velocity calculation numerator (Detlat * Co^2)
	VelocityCalcNumerator_m2perS = *TOF_DIFF_DeltaT_S * SpeedOfSound_mperS* SpeedOfSound_mperS;
	//Velocity calculation denominator is 2 * Length of path in flow
	VelocityCalcDenominatore_m = 2 * PIPELENGTH_IN_FLOW_M;
	Velocity_mPers = VelocityCalcNumerator_m2perS / VelocityCalcDenominatore_m;	//final divsion
	*FlowVelocity = Velocity_mPers;
	return success;
}

#define PIPERADIUS_M	0.008
bool Calculate_Volumetric_Flow( float Velocity_mPerS, float* VolumetricFlow_m3PerS, float* VolumetricGainfactor, float*VolumetricFlowCorrected_m3PerS)
{
	bool success = false;
	int XCord;
	float CrossSectionalArea_m2;
	//determine cross sectional area (PI*R*R)
	CrossSectionalArea_m2 = PI * PIPERADIUS_M * PIPERADIUS_M;

	//Determine volumetric flow for a given velocity ( Q = V * A )
	*VolumetricFlow_m3PerS = Velocity_mPerS * CrossSectionalArea_m2;

	//Find the Volumetric Flow Adjustment Gain Coefficient from the lookup Table
	//Find the X coordinate in the lookup table. XCord is the Meter Volumetric Flow Reading
	for (XCord = 0;	((VOLUMETRIC_FLOW_CORRECTION_TABLE[XCord][0] < *VolumetricFlow_m3PerS * SECONDS_PER_HR) ); XCord++)
			{;}//Purely searching for the index

	*VolumetricGainfactor = LinearInterpolation
											(
												VOLUMETRIC_FLOW_CORRECTION_TABLE[XCord-1][0],		//X1 - //Index was past
												VOLUMETRIC_FLOW_CORRECTION_TABLE[XCord][0],			//X2
												VOLUMETRIC_FLOW_CORRECTION_TABLE[XCord-1][1],		//Y1 //A de-referenced lookup table item is gain correction (unitless)
												VOLUMETRIC_FLOW_CORRECTION_TABLE[XCord][1],	//Y2
												(*VolumetricFlow_m3PerS * SECONDS_PER_HR)	//New X of interest
											);//Returns new Y

	*VolumetricFlowCorrected_m3PerS = (*VolumetricGainfactor) * (*VolumetricFlow_m3PerS);
	return success;
}


//todo: customer define their nominal 0C Platinum RTD resistance (PT1k = 1000 ohms)
		//todo:The radius of the pipe used to determine cross sectional area (PI*R*R) - update to spool body
bool Calculate_Energy_Parameters(Flow_ResultsStruct* TOF_DIFF_ResultsToUse, TemperatureResultsAllPortsStruct* TemperatureResultsToUse, Energy_ResultsStruct* EnergyToUse)
{
	bool success = false;
	Calculate_Mass_Flow(
			TOF_DIFF_ResultsToUse->VolumetricFlowCorrected_m3PerS,
			&EnergyToUse->MassFlow_kgPerh,
			TemperatureResultsToUse->TempResultsAllPorts[HOT_TEMP_PORT].TemperatureDegreeC
			);
	Calculate_Enthalpy(
			TemperatureResultsToUse->TempResultsAllPorts[COLD_TEMP_PORT].TemperatureDegreeC,
			TemperatureResultsToUse->TempResultsAllPorts[HOT_TEMP_PORT].TemperatureDegreeC,
			&EnergyToUse->EnthalpyAtCold_JperKg,
			&EnergyToUse->EnthalpyAtHot_JperKg,
			&EnergyToUse->EnthalpyDelta_JperKg
			);
	return success;
}

//Mass flow of the water flowing across the heat meter (kg/3)
bool Calculate_Mass_Flow(float VolumetricFlowCorrected_m3PerS, float* MassFlow_kgPerHr,	float Temperature)
{
	bool success = false;
	float WaterDensityatTemp_kgPerm3, VolumetricFlowCorrected_m3PerH;
	int TempAtJunncture = (int)(Temperature);	//Get the temperature of the water where the TOF measurement was taken, Truncate
	WaterDensityatTemp_kgPerm3 = LinearInterpolation 	//Current table is mapped with degrees C, just insert whole degree and read out density
											(
												TempAtJunncture,		//X1
												TempAtJunncture + 1,	//X2
												WATER_DENSITY_LOOKUPTABLE[TempAtJunncture],		//Y1 //A de-referenced lookup table item is density  kg/m3
												WATER_DENSITY_LOOKUPTABLE[TempAtJunncture + 1],	//Y2
												Temperature	//New X of interest
											);//Returns new Y
	//Determine Mass flow for a given volumetric flow rate and Density
	VolumetricFlowCorrected_m3PerH = VolumetricFlowCorrected_m3PerS * SECONDS_PER_HR;
	*MassFlow_kgPerHr = VolumetricFlowCorrected_m3PerH * WaterDensityatTemp_kgPerm3;
	return success;
}

bool Calculate_Enthalpy(float TemperatureAtCold, float TemperatureAtHot, float* EnthalpyAtCold_JperKg, float* EnthalpyAtHot_JperKg,	float* DeltaEnthalpy_JperKg)
{
	bool success = false;
	int TempAtJunncture = (int)(TemperatureAtCold);	//Get the temperature of the water where the TOF measurement was taken, Truncate
	*EnthalpyAtCold_JperKg = LinearInterpolation
											(
												TempAtJunncture,		//X1
												TempAtJunncture + 1,	//X2 (increment of 1 is next lookup value +1C)
												WATER_ENTHALPY_LOOKUPTABLE[TempAtJunncture],		//Y1 //A de-referenced lookup table item is enthalpy j/kg
												WATER_ENTHALPY_LOOKUPTABLE[TempAtJunncture + 1],	//Y2
												TemperatureAtCold	//New X of interest
											);//Returns new Y

	TempAtJunncture = (int)(TemperatureAtHot);	//Get the temperature of the water where the TOF measurement was taken, Truncate
	*EnthalpyAtHot_JperKg = LinearInterpolation
											(
												TempAtJunncture,		//X1
												TempAtJunncture + 1,	//X2 (increment of 1 is next lookup value +1C)
												WATER_ENTHALPY_LOOKUPTABLE[TempAtJunncture],		//Y1 //A de-referenced lookup table item is enthalpy j/kg
												WATER_ENTHALPY_LOOKUPTABLE[TempAtJunncture + 1],	//Y2
												TemperatureAtHot	//New X of interest
											);//Returns new Y
	//Calculate the The delta Enthalpy in J/Kg
	*DeltaEnthalpy_JperKg = *EnthalpyAtHot_JperKg - *EnthalpyAtCold_JperKg;
	return success;
}

void Calculate_TimeDifference( tm_withMilli*  t1, tm_withMilli*  t2, float* difference_sec)
{
	time_t t_t1, t_t2;
	float milliSecAdd = 0;

	t_t1 = mktime(&t1->Time);
	t_t2 = mktime(&t2->Time);
	*difference_sec = 	(float) difftime (t_t2,t_t1 );	//todo change to double

	if( t2->milliSeconds > t1->milliSeconds)
		milliSecAdd = t2->milliSeconds - t1->milliSeconds;
	else
	{
		milliSecAdd = (1000 + t2->milliSeconds) - t1->milliSeconds;
		*difference_sec = *difference_sec - 1; 	//there must be atlease a one second difference if t2
	}

	*difference_sec = *difference_sec + (milliSecAdd/1000);
}

float energyAddition[30] , energyAddition1;
int energyAdditionsTracker= 0;
bool Calculate_Piecewiese_Energy(PointOfTimeSampleDataStruct* POT1, PointOfTimeSampleDataStruct* POT2, float *EnergyForTime_J)
{
	bool success = false;
	float TimeDifference_sec = 0;

	Calculate_TimeDifference(&POT1->POT_TimeData, &POT2->POT_TimeData,	&TimeDifference_sec);
	energyAddition1 = POT1->POT_EnergyFactors.EnthalpyDelta_JperKg * POT1->POT_EnergyFactors.MassFlow_kgPerh * TimeDifference_sec/3600;
//	energyAddition[energyAdditionsTracker++] = energyAddition1;
	*EnergyForTime_J += energyAddition1;
	return success;
}

float volumeAddition[30] , volumeAddition1;
int volueAdditionsTracker= 0;
bool Calculate_Piecewise_Volume(PointOfTimeSampleDataStruct* POT1, PointOfTimeSampleDataStruct* POT2, float *Volume_m3)
{
	bool success = false;
	float TimeDifference_sec = 0;

	Calculate_TimeDifference(&POT1->POT_TimeData, &POT2->POT_TimeData,	&TimeDifference_sec);
	volumeAddition1 = POT1->POT_FlowFactors.VolumetricFlowCorrected_m3PerS * TimeDifference_sec;
	//volumeAddition[volueAdditionsTracker++] = volumeAddition1;
	*Volume_m3 += volumeAddition1;
	return success;
}


//generic interpolation routine
float LinearInterpolation (float X1Low, float X2High, float Y1Low, float Y2High, float XnewLookup)
{
	float XnewLookupRatio, Xrange, Yrange, YReturnLookupValue;
	Xrange = (X2High-X1Low);
	Yrange = (Y2High-Y1Low);
	XnewLookupRatio = (XnewLookup - X1Low)/ Xrange;
	YReturnLookupValue = Y1Low + Yrange*XnewLookupRatio;
	return (YReturnLookupValue);
}


int FloatToString (float fNumber, uint8_t precision, char* output)
{
	int base = 1;
	int length1 = 0, length2 = 0;
	for (int loop = 0; loop < precision; loop++)
		base *= 10;
	fNumber > 0? (fNumber += (0.5/base)) : (fNumber -= (0.5/base));

	length1 = snprintf(output, 16, "%d", (int)fNumber);

	if(precision <= 0)
		return length1;
	else
	{
		fNumber > 0? (fNumber = fNumber - (int)fNumber) : (fNumber = (int)fNumber - fNumber);
		fNumber *= base;

		output[length1] = '.';
		length2 = snprintf(&output[length1+1], 16, "%0*d", precision, (int)fNumber);

		return length1 + length2 + 1;

	}
}

/*******************************  END - Meter Calculation routines*********************************/


/*******************************  MAX35101 Getting results data from MAX35101 *********************************/
bool MAX35101_Update_TemperatureData(TemperatureResultsAllPortsStruct* TempResultsToUpdate)
{
	bool success = false;

	//Update Temp Structure RREF
	success |=
			MAX35101_Read_2WordValue(T3_REG,
					&TempResultsToUpdate->TempResultsAllPorts[REFERENCE_TEMP_PORT].Register_Value);	//Read the interrupt register

	//Update Temp Structure1
	success |=
			MAX35101_Read_2WordValue(T1_REG,
					&TempResultsToUpdate->TempResultsAllPorts[HOT_TEMP_PORT].Register_Value);//Read the interrupt register
	Calcuate_Temperature(
			TempResultsToUpdate->TempResultsAllPorts[HOT_TEMP_PORT].Register_Value,
			TempResultsToUpdate->TempResultsAllPorts[REFERENCE_TEMP_PORT].Register_Value,
			&TempResultsToUpdate->TempResultsAllPorts[HOT_TEMP_PORT].TemperatureDegreeC);

	//Update Temp Structure2
	success |=
			MAX35101_Read_2WordValue(T2_REG,
					&TempResultsToUpdate->TempResultsAllPorts[COLD_TEMP_PORT].Register_Value);//Read the interrupt register
	Calcuate_Temperature(
			TempResultsToUpdate->TempResultsAllPorts[COLD_TEMP_PORT].Register_Value,
			TempResultsToUpdate->TempResultsAllPorts[REFERENCE_TEMP_PORT].Register_Value,
			&TempResultsToUpdate->TempResultsAllPorts[COLD_TEMP_PORT].TemperatureDegreeC);

	return success;
}


bool MAX35101_Update_TOF_AVG_DIFFData(Flow_ResultsStruct* TOF_DIFF_Results)
{
	bool success = false;
	//Update TOF Diff Average (This is what to read out in Event Timing mode)
		success |= MAX35101_Read_2WordValue(TOF_DIFF_AVG_REG, &TOF_DIFF_Results->TOF_DiffData);

	return success;
}

bool MAX35101_Update_TOF_DIFFData(Flow_ResultsStruct* TOF_DIFF_Results)
{
	bool success = false;

#ifndef USING_EVENT_TIMING_MODES_READ_AVERAGE
	//Update TOF Down
	success |= MAX35101_Read_2WordValue(HIT1DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit1Data);
	success |= MAX35101_Read_2WordValue(HIT2DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit2Data);
	success |= MAX35101_Read_2WordValue(HIT3DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit3Data);
	success |= MAX35101_Read_2WordValue(HIT4DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit4Data);
	success |= MAX35101_Read_2WordValue(HIT5DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit5Data);
	success |= MAX35101_Read_2WordValue(HIT6DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit6Data);
	success |= MAX35101_Read_2WordValue(HIT6DOWN_REG,
			&TOF_DIFF_Results->HitDownData.Hit6Data);
	success |= MAX35101_Read_2WordValue(AVGDOWN_REG,
			&TOF_DIFF_Results->HitDownData.HitAverageData);

	//Update TOF Up
	success |= MAX35101_Read_2WordValue(HIT1UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit1Data);
	success |= MAX35101_Read_2WordValue(HIT2UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit2Data);
	success |= MAX35101_Read_2WordValue(HIT3UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit3Data);
	success |= MAX35101_Read_2WordValue(HIT4UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit4Data);
	success |= MAX35101_Read_2WordValue(HIT5UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit5Data);
	success |= MAX35101_Read_2WordValue(HIT6UP_REG,
			&TOF_DIFF_Results->HitUpData.Hit6Data);
	success |= MAX35101_Read_2WordValue(AVGUP_REG,
			&TOF_DIFF_Results->HitUpData.HitAverageData);

	//Update TOF Diff
	success |= MAX35101_Read_2WordValue(TOF_DIFF_REG, &TOF_DIFF_Results->TOF_DiffData);
#else
	//Update TOF Diff
		success |= MAX35101_Read_2WordValue(TOF_DIFF_REG, &TOF_DIFF_Results->TOF_DiffData);
#endif

	return success;
}

//Read the RTC values from the MAX35101 and updat a time data structure pointed to by the funtion call pass parameter
bool MAX35101_UpdateAndGetTime(tm_withMilli* RTCTimeStamp) {
	bool success = false;
	uint16_t readData = 0;

	success = MAX35101_Read_Register(SECONDS, &readData);
	RTCTimeStamp->Time.tm_sec = ((((readData & 0x00F0) >> 4 ) * 10) + (readData & 0x000F));
	RTCTimeStamp->milliSeconds = ((((readData & 0xF000) >> 12 ) * 100) + ((readData & 0x0F00)>>8) *10 );

	success |= MAX35101_Read_Register(MINS_HRS, &readData);
	RTCTimeStamp->Time.tm_min = ((( readData & 0xF000) >> 12) * 10) + ((readData & 0x0F00) >> 8);
	//RTCTimeStamp->Time.tm_hour= (((( readData & 0x0030) >> 4)   * 20) + ((readData & 0x0008) >> 3) * 10) + (readData & 0x0007);
	RTCTimeStamp->Time.tm_hour= ((( readData & 0x0030) >> 4)   * 10) + (readData & 0x000F);

	success |= MAX35101_Read_Register(DAY_DATE, &readData);
	RTCTimeStamp->Time.tm_mday = ((((readData & 0x0030) >> 4 ) * 10) + (readData & 0x000F));

	success |= MAX35101_Read_Register(MONTH_YEAR, &readData);
	RTCTimeStamp->Time.tm_mon = ((( readData & 0x1000) >> 12) * 10) + ((readData & 0x0F00) >> 8);
	RTCTimeStamp->Time.tm_year = ((((readData & 0x00F0) >> 4 ) * 10) + (readData & 0x000F)) + 100;	//tm_year is years since 1900

	return success;
}

bool MAX35101_Enable_LDO(void)
{
	MAX35101_Send_Opcode(LDO_ON);
	Delay(1);	//Blind Wait LDO stabilize
	return true;
}

bool MAX35101_Disable_LDO(void)
{
	MAX35101_Send_Opcode(LDO_OFF);
	return true;
}

bool MAX35101_Write_Flash(uint16_t StartingFlashAddress, uint16_t WriteLength, uint16_t* DataToWrite)
{
	bool success;
	int i;
	uint16_t FlashAddressToWrite;

	MAX35101_Enable_LDO();
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	for ( i = 0; i < WriteLength; i++ )
	{
		FlashAddressToWrite = StartingFlashAddress + (2*i);	//Need to increment the address since it is only 1 word writing capability
		MAX35101_CS_Low();					//Modulate CS#
		success = SPI_Send_Byte(WRITE_FLASH);		//Transmit the write flash command code
		success = SPI_Send_Byte((char) (FlashAddressToWrite >> 8));		//Transmit the Flash Address (MUST be Even)
		success = SPI_Send_Byte((char) (FlashAddressToWrite & 0x00FF));	//Transmit the 8 bits
		success = SPI_Send_Byte((char) (( *(DataToWrite + i) ) >> 8));		//Transmit the Data
		success = SPI_Send_Byte((char) (( *(DataToWrite + i) & 0x00FF)));	//Transmit the 8 bits
		MAX35101_CS_High();					//Modulate CS#
		Delay (1);	//Blind 1 ms delay for word write time
	}
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	MAX35101_Disable_LDO();
	return success;
}

bool MAX35101_Read_Flash(uint16_t FlashAddress, uint16_t ReadLength, uint16_t* DataToRead)
{
	bool success;
	int i;

	MAX35101_Enable_LDO();
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(READ_FLASH);		//Transmit the read flash command code
	success = SPI_Send_Byte((char) (FlashAddress >> 8));		//Transmit the Flash Address (MUST be Even)
	success = SPI_Send_Byte((char) (FlashAddress & 0x00FF));	//Transmit the 8 bits
	for ( i = 0; i < ReadLength; i++ )
	{
		success = SPI_Read_Word( (DataToRead + i) );		//Read
	}
	MAX35101_CS_High();					//Modulate CS#
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	MAX35101_Disable_LDO();
	return success;
}

bool MAX35101_BlockErase_Flash(uint16_t AddressBlock)
{
	bool success;
	MAX35101_Enable_LDO();
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(BLOCK_ERASE_FLASH);		//Transmit the read Block Erase command code
	success = SPI_Send_Byte((char) (AddressBlock >> 8));		//Transmit the Block Address
	success = SPI_Send_Byte((char) (AddressBlock & 0x00FF));	//Transmit the 8 bits
	MAX35101_CS_High();					//Modulate CS#
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	Delay(60);
	MAX35101_Disable_LDO();

	return success;
}
/*******************************  END - MAX35101 Getting results data from MAX35101 *********************************/
uint32_t flags;
// GPIO Interrupt handler
void GPIO_ODD_IRQHandler(void)
{
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	  // Get interrupts flags.
	  flags = GPIO_IntGet();

	  if(flags & (1<< 7))   	// SW4 is pressed
	  {
		  //todo: implement factory date/time settings, work together with SW3.
		  GPIO_IntClear(1 << 7);
	  }
	  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

// GPIO Interrupt handler
// System will be in micro deep sleep with the MAX35101 running an infinite event timing mode
// Upon temperature completion of a EVTMG sequence or TOF sequence completion the INT#
// pin of the MAX35101 becomes active and the data must be read by my micro
uint16_t InterruptRegisterValue;

void GPIO_EVEN_IRQHandler(void)
{
	uint8_t pinState = 0;
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	// Get interrupts flags.
	flags = GPIO_IntGet();
	// Process interrupts based on interrupt resources.
	// SW2 is pressed, display information.
	if(flags & (1 << 0))
	{
		delayCount = 0;
		while(!GPIO_PinInGet(gpioPortA, 0))
			delayCount += 1;

		if(delayCount < 3)		    // not a real press
			;
		else if(delayCount < 1000000)	// good, normal press
		{
			DisplayMode++;
			displayRepeat = 0;
			if (DisplayMode == Display_Last)
				DisplayMode = Display_Welcome;
		}
		// long press in TDF, TDM, TMF, TMM modes means to change register.
		//else if((DisplayMode == Display_TDF_Config) | (DisplayMode == Display_TDM_Config) | (DisplayMode == Display_TMF_Config) |(DisplayMode == Display_TMM_Config))

		else if(DisplayMode == Display_TDF_Config)
		{
			DOGM163_ClearChars(0, 0, 48);
			DOGM163_PrintChars(0, 0, 16, "Change TDF[3:0]:");
			FloatToString((TDF + 1)*0.5, 1, output);
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 9, 7, "Seconds");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Accept");

			while(1)
			{
				delayCount = 0;
				while(!GPIO_PinInGet(gpioPortA, 0))
					delayCount += 1;

				if(delayCount < 3)		    	// not a real press
						;
				else if(delayCount < 1000000)	// good, normal press
				{
					if(TDF == 0x0F)
						TDF = 0x00;
					else
						TDF++;
					FloatToString((TDF + 1)*0.5, 1, output);
					DOGM163_PrintChars(1, 0, 16, "   ");	// clear 3 digits for TDF.
					DOGM163_PrintChars(1, 0, 16, output);
				}
				else							// accept the value.
				{
					MAX35101_Read_Register(0x3F,&reg);
					reg = (reg & 0x0FFF) | (TDF << 12);
					MAX35101_Write_Register(0x3F, reg);

					//DisplayMode++;
					displayRepeat = 0;
					break;
				}
			}
		}

		else if(DisplayMode == Display_TDM_Config)
		{
			DOGM163_ClearChars(0, 0, 48);
			DOGM163_PrintChars(0, 0, 16, "Change TDM[4:0]:");
			snprintf(output, 16, "%d", TDM + 1);
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 10, 6, "Cycles");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Accept");

			while(1)
			{
				delayCount = 0;
				while(!GPIO_PinInGet(gpioPortA, 0))
					delayCount += 1;

				if(delayCount < 3)		    	// not a real press
						;
				else if(delayCount < 1000000)	// good, normal press
				{
					if(TDM == 0x1F)
						TDM = 0x00;
					else
						TDM++;
					snprintf(output, 16, "%d", TDM + 1);
					DOGM163_PrintChars(1, 0, 16, "  ");	//clear 2 digits for TDM.
					DOGM163_PrintChars(1, 0, 16, output);
				}
				else							// accept the value.
				{
					MAX35101_Read_Register(0x3F,&reg);
					reg = (reg & 0xF07F) | (TDM << 7);
					MAX35101_Write_Register(0x3F, reg);

					//DisplayMode++;
					displayRepeat = 0;
					break;
				}
			}
		}

		else if(DisplayMode == Display_TMF_Config)
		{
			DOGM163_ClearChars(0, 0, 48);
			DOGM163_PrintChars(0, 0, 16, "Change TMF[5:0]:");
			snprintf(output, 16, "%d", TMF + 1);
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 9, 7, "Seconds");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Accept");

			while(1)
			{
				delayCount = 0;
				while(!GPIO_PinInGet(gpioPortA, 0))
					delayCount += 1;

				if(delayCount < 3)		    	// not a real press
						;
				else if(delayCount < 1000000)	// good, normal press
				{
					if(TMF == 0x3F)
						TMF = 0x00;
					else
						TMF++;
					snprintf(output, 16, "%d", TMF + 1);
					DOGM163_PrintChars(1, 0, 16, "  ");	//clear 2 digits for TMF.
					DOGM163_PrintChars(1, 0, 16, output);
				}
				else							// accept the value.
				{
					MAX35101_Read_Register(0x3F,&reg);
					reg = (reg & 0xFF80) | (TMF << 1);
					MAX35101_Write_Register(0x3F, reg);

					//DisplayMode++;
					displayRepeat = 0;
					break;
				}
			}
		}

		else if(DisplayMode == Display_TMM_Config)
		{
			DOGM163_ClearChars(0, 0, 48);
			DOGM163_PrintChars(0, 0, 16, "Change TMM[4:0]:");
			snprintf(output, 16, "%d", TMM + 1);
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 10, 6, "Cycles");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Accept");

			while(1)
			{
				delayCount = 0;
				while(!GPIO_PinInGet(gpioPortA, 0))
					delayCount += 1;

				if(delayCount < 3)		    	// not a real press
						;
				else if(delayCount < 1000000)	// good, normal press
				{
					if(TMM == 0x1F)
						TMM = 0x00;
					else
						TMM++;
					snprintf(output, 16, "%d", TMM + 1);
					DOGM163_PrintChars(1, 0, 16, "  ");	//clear 2 digits for TMM.
					DOGM163_PrintChars(1, 0, 16, output);
				}
				else							// accept the value.
				{
					MAX35101_Read_Register(0x40,&reg);
					reg = (reg & 0x07FF) | (TMM << 11);
					MAX35101_Write_Register(0x40, reg);

					//DisplayMode++;
					displayRepeat = 0;
					break;
				}
			}
		}
		else if((delayCount > 10000000) & !EVT_STARTED)		// if not started, long press to activate the meter.
		{
			DOGM163_ClearChars(0, 0, 48);
			DOGM163_PrintChars(0, 0, 16, "Activating");
			DOGM163_PrintChars(1, 0, 16, "The");
			DOGM163_PrintChars(2, 0, 16, "Meter...");
			MAX35101_Send_Opcode(EVTMG1);
			EVT_STARTED = true;
			for(int loop=0; loop<10000000; loop++)
			{
				pinState = GPIO_PinInGet(gpioPortA, 0);
			}
		}
		else 											// 1000,000 to 10,000,000, turn off the display
		{
			displayRepeat = MAX_DISPLAY_REPEAT + 1;
		}

		GPIO_IntClear(1 << 0);				// Acknowledge interrupt
	 }

	//Interrupt# Pin from MAX35101. Read the Interrupt Status Register and then read associated register data.
	else if(flags & (1 << 2))
	{
		MAX35101_Read_Register(INTERRUPT_STATUS, &InterruptRegisterValue);
		if ((InterruptRegisterValue & INTERRUPT_REG_TO) != 0x00)	//Timeout has occurred
		{
			// todo: timeout error handling.
		}

		// Temp Sequence is completed
		if ((InterruptRegisterValue & INTERRUPT_REG_TEMP_EVTMG) != 0x00)
		{
			//Get all the Temp data and update the Last Temp structure.
			MAX35101_Update_TemperatureData(&Last_TempUpdate);
		}

		// TOF Sequence is completed.
		if ((InterruptRegisterValue & INTERRUPT_REG_TOF_EVTMG) != 0x00)
		{
			// Get all the TOF Data and update the Point-in-Time Snap shot.
			MAX35101_Update_TOF_AVG_DIFFData(&POT_Data[POTCount].POT_FlowFactors);
			// Get all the Time Data and update the Point-in-Time Snap shot.
			MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
			// Copy the most recent Temp Data into Point-in-time snap shots.
			POT_Data[POTCount].POT_TemperatureData = Last_TempUpdate;

			// Do all the math calculations on the Raw Data found in the structures.
			Calculate_Flow_Parameters(&POT_Data[POTCount].POT_FlowFactors, &POT_Data[POTCount].POT_TemperatureData);
			Calculate_Energy_Parameters(&POT_Data[POTCount].POT_FlowFactors, &POT_Data[POTCount].POT_TemperatureData, &POT_Data[POTCount].POT_EnergyFactors);
			// Calculate volume for two given points in time data sets.
			Calculate_Piecewise_Volume (&POT_Data_Last, &POT_Data[POTCount], &TotalVolume_m3);
			// Calculate energy for two given points in time data sets.
			Calculate_Piecewiese_Energy(&POT_Data_Last, &POT_Data[POTCount], &TotalEnergy);
			POT_Data_Last = POT_Data[POTCount];
		}

		// Acknowledge interrupt.
		GPIO_IntClear(1 << 2);
	}

	else if(flags & (1<< 14))   	// SW3 is pressed
	{
		// Pin press debouncing. Just do something to delay.
		for(int loop=0; loop<1000; loop++)
		{
			pinState = GPIO_PinInGet(gpioPortC, 14);
		}

		// Check if SW3 is still pressed. Works as a debouncing filter.
		if(!pinState && DisplayMode)
		{
			// Factory date/time setting.
			MAX35101_SetTime();
		}
		// Acknowledge interrupt.
		GPIO_IntClear(1 << 14);
	}
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

int main(void)
{

	CHIP_Init();		// Specific Micro Chip errata

	// Setup SysTick Timer for 1 msec interrupts
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		while (1);
	}

	init();	//more micro specific initialization routines


	MAX35101_Send_Opcode(RESET);	//Blindly send the MAX35101 reset, in case it is currently running an Event Timing mode
	Delay(100);	//Blind wait
	MAX35101_Send_Opcode(INITIALIZE);	//Send the initialization command, see datasheet
	Delay(100);	//Blind wait

	MAX35101_Write_Register(MINS_HRS, 0x5923);	//Set the time
	MAX35101_Write_Register(DAY_DATE, 0x0019);	//Set the Date
	MAX35101_Write_Register(MONTH_YEAR, 0x0914);//Set the year

	// start infinite loop.
	//MAX35101_Send_Opcode(EVTMG1);

	while(1)
	{
		if(DisplayMode != Display_Off)
			Display();
		else
			EMU_EnterEM3(false);

	}
}

void Display(void)
{
	int POTCountShown;	// Which is the LAST valid data point taken, this is what we will show

	POTCountShown = POTCount;

	//only update the screen twice a second.
	Delay(500);

	// Refresh the screen during normal display modes.
	DOGM163_ClearChars(0, 0, 48);

	// Power on and initialize the display if display power is off.
	if(displayPowerOff)
	{
		DOGM163_Init();
		displayPowerOff = false;
	}

	// Power off the display if the same info has been displayed for MAX_DISPLAY_REPEAT times.
	if(DisplayMode == previousDisplayMode)
	{
		displayRepeat ++;
		if(displayRepeat > MAX_DISPLAY_REPEAT)
		{
			DOGM163_PowerOff();
			displayPowerOff = true;
			displayRepeat = 0;
			DisplayMode = Display_Off;
			previousDisplayMode = Display_Off;
			return;
		}
	}

	switch (DisplayMode)
	{
		case Display_Welcome:
			DOGM163_PrintWelcomeMsg();
			previousDisplayMode = Display_Welcome;
			break;
		case Display_Clock:
			MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);

			DOGM163_PrintChars(0, 0, 16, "Real Time Clock:");
			sprintf(output, "%02d/%02d/%04d",
					POT_Data[POTCountShown].POT_TimeData.Time.tm_mon,
					POT_Data[POTCountShown].POT_TimeData.Time.tm_mday,
					POT_Data[POTCountShown].POT_TimeData.Time.tm_year + 1900);
			DOGM163_PrintChars(1, 0, 16, output);

			sprintf(output, "%02d:%02d:%02d",
					POT_Data[POTCountShown].POT_TimeData.Time.tm_hour,
					POT_Data[POTCountShown].POT_TimeData.Time.tm_min,
					POT_Data[POTCountShown].POT_TimeData.Time.tm_sec);
			DOGM163_PrintChars(2, 0, 16, output);
			previousDisplayMode = Display_Clock;
			break;
		case Display_Temp:
			DOGM163_PrintChars(0, 0, 16, "RTD Temperature:");

			sprintf(output, "RTD1:");
			FloatToString(POT_Data[POTCountShown].POT_TemperatureData.TempResultsAllPorts[HOT_TEMP_PORT].TemperatureDegreeC, 3, &output[5]);
			strcat(output, " C");
			DOGM163_PrintChars(1, 0, 16, output);

			sprintf(output, "RTD2:");
			FloatToString(POT_Data[POTCountShown].POT_TemperatureData.TempResultsAllPorts[COLD_TEMP_PORT].TemperatureDegreeC, 3, &output[5]);
			strcat(output, " C");
			DOGM163_PrintChars(2, 0, 16, output);

			previousDisplayMode = Display_Temp;
			break;
		case Display_TOFDIFF:

		    DOGM163_PrintChars(0, 0, 16, "TOF Difference:");
			sprintf(output, "%d",(int) (POT_Data[POTCountShown].POT_FlowFactors.TOF_DIFF_DeltaT_S* 1000000000000));
            DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(2, 0, 16, "ps");
			previousDisplayMode = Display_TOFDIFF;
			break;

		case Display_Volumetric_Flow:

			DOGM163_PrintChars(0, 0, 16, "Flow Rate:");
			FloatToString(POT_Data[POTCountShown].POT_FlowFactors.VolumetricFlowCorrected_m3PerS* 60 * 1000, 3, output);
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(2, 0, 16, "LPM");
			previousDisplayMode = Display_Volumetric_Flow;
			break;

		case Display_Total_Volume:

			DOGM163_PrintChars(0, 0, 16, "Totalizer:");
			FloatToString(TotalVolume_m3 * 1000, 3, output);					// 1 m3 = 1000 liters.
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(2, 0, 16, "Liters");
			previousDisplayMode = Display_Total_Volume;
			break;

		case Display_Energy:
			DOGM163_PrintChars(0, 0, 16, "Total Energy:");
			snprintf(output, 16, "%d", (int) (TotalEnergy * 0.00094781712));	// 1 BTU = 1055.05585 joules.
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(2, 0, 16, "BTU");
			previousDisplayMode = Display_Energy;
			break;

		case Display_TDF_Config:
			MAX35101_Read_Register(0x3F,&reg);
			TDF = (reg & 0xF000) >> 12;
			FloatToString((TDF + 1)*0.5, 1, output);
			DOGM163_PrintChars(0, 0, 16, "TDF[3:0] Reg");
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 9, 7, "Seconds");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Change");
			previousDisplayMode = Display_TDF_Config;

			break;

		case Display_TDM_Config:
			MAX35101_Read_Register(0x3F,&reg);
			TDM = (reg & 0x0F80) >> 7;
			snprintf(output, 16, "%d", TDM + 1);
			DOGM163_PrintChars(0, 0, 16, "TDM[4:0] Reg");
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 10, 6, "Cycles");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Change");
			previousDisplayMode = Display_TDM_Config;

			break;

		case Display_TMF_Config:
			MAX35101_Read_Register(0x3F,&reg);
			TMF = (reg & 0x007E) >> 1;
			snprintf(output, 16, "%d", TMF + 1);
			DOGM163_PrintChars(0, 0, 16, "TMF[5:0] Reg");
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 9, 7, "Seconds");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Change");
			previousDisplayMode = Display_TMF_Config;

			break;

		case Display_TMM_Config:
			MAX35101_Read_Register(0x40,&reg);
			TMM = (reg & 0xF800) >> 11;
			snprintf(output, 16, "%d", TMM + 1);
			DOGM163_PrintChars(0, 0, 16, "TMM[4:0] Reg");
			DOGM163_PrintChars(1, 0, 16, output);
			DOGM163_PrintChars(1, 10, 6, "Cycles");
			DOGM163_PrintChars(2, 0, 16, "LongPress:Change");
			previousDisplayMode = Display_TMM_Config;

			break;

		default:
			break;
	}
}


/*******************************  MAX35101 SPI sequence Specific SPI ROUNTINES *********************************/
bool MAX35101_SendConfigs(void)	//need to rewrite
{

	MAX35101_Write_Register(0x38, 0x1311);
	MAX35101_Write_Register(0x39, 0xA3F2);
	MAX35101_Write_Register(0x3A, 0x0809);
	MAX35101_Write_Register(0x3B, 0x0A0B);
	MAX35101_Write_Register(0x3C, 0x0C0D);
	MAX35101_Write_Register(0x3D, 0x0048);
	MAX35101_Write_Register(0x3E, 0x0048);


	MAX35101_Write_Register(0x3F, 0x0180); 			// TOF_DIFF: 0.5s, 4 cycles, Temp: 1s, 2 cycles.
	MAX35101_Write_Register(0x40, 0x0FCF);			// Use calibration, at the beginning of TOF_DIFF and TEMP sequences.
													// Measure T1, T2, T3. PRECYC = 3, PORTCYC = 512us.


	MAX35101_Write_Register(0x41, 0x0108);
	MAX35101_Write_Register(0x42, 0x0343);	// CMP_EN=0, CMP_SEL=0, INT_EN=1, ET_CONT=1, CONT_INT=0, clock settling=5.13ms, 4MHz calibration=4 32kHz cycles.
	MAX35101_Write_Register(0x43, 0x0080);	// 32K_BP=0, 32K_EN=0(no 32KOUT), enable 32kHz, no alarm, no watch dog.

	MAX35101_Send_Opcode(TRANSFER_TO_FLASH);
	Delay(1000);
	return true;
}

bool MAX35101_Send_Opcode(char opcode)//Single Byte Opcode Send ONLY (Modulates the CS# line)
{
	bool success;
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(opcode);	//Transmit the 8 bits over the SPI Bus
	MAX35101_CS_High();					//Modulate CS#
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	return success;
}

bool MAX35101_Read_2WordValue(char startingAddress, uint32_t* results)//Read two words that make up a temperature reading or Average, TOF read or waverage, Modulates CS# line, Sends address, reads 32 bits data
{
	bool success = false;
	uint16_t Readresults1, Readresults2 = 0;

	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	startingAddress |= 0x80;//Read Opcodes Always have MSB of Opcode (address) Set
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(startingAddress);//Transmit the 8 bits over the SPI Bus
	success |= SPI_Read_Word(&Readresults1);
	success |= SPI_Read_Word(&Readresults2);
	MAX35101_CS_High();					//Modulate CS#
	*results = (((uint32_t) Readresults1 << 16) + Readresults2);//Combine the 2 words into one 32 bits number
	success = false;
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	return success;
}

bool MAX35101_Read_Register(char address, uint16_t* results)//Read one word register only, Modulates CS# line, Sends address, reads data
{
	bool success;
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	address |= 0x80;	//Read Opcodes Always have MSB of Opcode (address) Set
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(address);	//Transmit the 8 bits over the SPI Bus
	success |= SPI_Read_Word(results);
	MAX35101_CS_High();					//Modulate CS#
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	return success;
}

bool MAX35101_Write_Register(char address, uint16_t DatatoWrite)// one word register only, Modulates CS# line, Sends address, reads data
{
	bool success;
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	MAX35101_CS_Low();					//Modulate CS#
	success = SPI_Send_Byte(address);	//Transmit the 8 bits over the SPI Bus
	success = SPI_Send_Byte((char) (DatatoWrite >> 8));	//Transmit the 8 bits over the SPI Bus
	success = SPI_Send_Byte((char) (DatatoWrite & 0x00FF));	//Transmit the 8 bits over the SPI Bus
	MAX35101_CS_High();					//Modulate CS#
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	return success;
}
/*******************************  END - MAX35101 SPI sequence Specific SPI ROUNTINES *********************************/


/*******************************  Micro Specific Routines and SPI Block ROUNTINES *********************************/

void init(void)
{
	/* Enable clocks to USART1 and GPIO. */
	CMU_ClockEnable(cmuClock_USART1, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Set MAXREFDES70 GPIOs to normal working mode. */
	GpioSetup();

	/* The design by default sample at the trailing edge (CLKPHA = USART_CTRL_CLKPHA); */
	SPI_setup(1, 0, true);

	// Initialize the DOGM163 display.
	DOGM163_Init();

	// Print welcome message for 2 seconds.
	DOGM163_PrintWelcomeMsg();

	//char output[16];

    //FloatToString(3.5, 3, output);

	Delay(5000);


	DOGM163_PowerOff();

	MAX35101_SendConfigs();	//Send standard configurations to MAX35101, these are ALL stored in flash and therefore don't need to be sent if the FLASH is preprogrammed
}

/**************************************
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *************************************/
void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
}

/***************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ********************************************/
extern void Delay(uint32_t dlyTicks) {
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks)
		;
}


#define HFRCO_FREQUENCY         14000000
#define SPI_PERCLK_FREQUENCY    HFRCO_FREQUENCY
#define SPI_BAUDRATE            1000000
#define NO_RX                    0
#define NO_TX                    NO_RX

void USART2_sendBuffer(char* txBuffer, int bytesToSend) {
	USART_TypeDef *uart = USART1;
	int ii;
	/* Sending the data */
	for (ii = 0; ii < bytesToSend; ii++) {
		/* Waiting for the usart to be ready */
		while (!(uart->STATUS & USART_STATUS_TXBL))
			;

		if (txBuffer != 0) {
			/* Writing next byte to USART */
			uart->TXDATA = *txBuffer;
			txBuffer++;
		} else {
			uart->TXDATA = 0;
		}
	}
	/*Waiting for transmission of last byte */
	while (!(uart->STATUS & USART_STATUS_TXC))
		;
}

bool SPI_Send_Byte(char dataByte)//Change this implementation to your Hardware
{
	bool success = true;
	//SPI2_setupRXInt(NO_RX, NO_RX);			/* Setting up RX interrupt for master */
	USART2_sendBuffer(&dataByte, 1); /* Transmitting data */
	return success;
}

bool SPI_Read_Word(uint16_t* results)//Change this implementation to your Hardware
{
	bool success = true;
	USART_TypeDef *spi = USART1;
	uint8_t rxdataByte1, rxdataByte2;
	spi->CMD = USART_CMD_CLEARRX;		//Clear the receive Buffer
	USART2_sendBuffer(NO_TX, 1); /* Receiving data by transmitting dummy data to slave */
	rxdataByte1 = spi->RXDATA;
	USART2_sendBuffer(NO_TX, 1); /* Receiving data by transmitting dummy data to slave */
	rxdataByte2 = spi->RXDATA;
	*results = (uint16_t) (rxdataByte2 + (rxdataByte1 << 8));
	return success;
}

/**************************************************************************//**
 * @brief Setup a USART as SPI
 * @param[in]	spiNumber	The number of the USART to use (e.g. 1 USART1)
 * @param[in]   location    The GPIO location to use for the device
 * @param[in]   master 		If the SPI is to be master
 *****************************************************************************/
static void SPI_setup(uint8_t spiNumber, uint8_t location, bool master) {
	USART_TypeDef *spi;
	GPIO_Mode_TypeDef gpioModeMosi;
	GPIO_Mode_TypeDef gpioModeMiso;
	// GPIO_Mode_TypeDef gpioModeCs  ;
	GPIO_Mode_TypeDef gpioModeClk;

	/* Determining USART */
	spi = USART1;

	/* Setting baudrate */
	spi->CLKDIV = 128 * (SPI_PERCLK_FREQUENCY / SPI_BAUDRATE - 2); //SPI_BAUDRATE

	/* Configure SPI */
	/* Using synchronous (SPI) mode*/
	spi->CTRL = USART_CTRL_SYNC | USART_CTRL_MSBF;       // | USART_CTRL_CLKPHA;
	/* Clearing old transfers/receptions, and disabling interrupts */
	spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	spi->IEN = 0;
	/* Enabling pins and setting location */
	//spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | (location << 8);
	spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN
			| (location << 8);

	/* Set to master and to control the CS line */
	if (master) {
		/* Enabling Master, TX and RX */
		spi->CMD = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
		//  spi->CTRL |= USART_CTRL_AUTOCS;

		/* Set GPIO config to master */
		gpioModeMosi = gpioModePushPull;
		gpioModeMiso = gpioModeInput;
		//  gpioModeCs   = gpioModePushPull;
		gpioModeClk = gpioModePushPull;
	} else {
		/* Enabling TX and RX */
		spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
	}

	/* Clear previous interrupts */
	spi->IFC = _USART_IFC_MASK;

	/* IO configuration */
	GPIO_PinModeSet(gpioPortC, 0, gpioModeMosi, 0); /* MOSI */
	GPIO_PinModeSet(gpioPortC, 1, gpioModeMiso, 0); /* MISO */
//  GPIO_PinModeSet(gpioPortC, 14, gpioModeCs,   0); /* CS */
	GPIO_PinModeSet(gpioPortB, 7, gpioModeClk, 0); /* Clock */
}



/**************************************************************************//**
 * @brief   Initialize the GPIO for the MAXREFDES70.
 *
 * @detail  This function initializes all resources required to support the
 *          GPIO functions for the MAXREFDES70.
 *****************************************************************************/
static void GpioSetup(void) {
	/* DOGM163 CS pin (PB11). */
	GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);
	/* DOGM163 RS pin (PB13). */
	GPIO_PinModeSet(gpioPortB, 13, gpioModePushPull, 0);

	/* MAX35101 COM_OUT/UP_DN pin (PC15). */
	GPIO_PinModeSet(gpioPortC, 15, gpioModeInput, 0);
	GPIO_IntConfig(gpioPortC, 15, false, true, false);
	/* MAX35101 WDO pin (PE12). */
	GPIO_PinModeSet(gpioPortE, 12, gpioModeInputPullFilter, 1);
	GPIO_IntConfig(gpioPortE, 12, false, true, false);
	/* MAX35101 INT pin (PF2). */
	GPIO_PinModeSet(gpioPortF, 2, gpioModeInputPullFilter, 1);
	GPIO_IntConfig(gpioPortF, 2, false, true, true );

	/* MAX35101 RST pin (PE13). */
	GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 1);
	/* MAX35101 CS pin (PB8). */
	GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1);

	/* DOGM163 power switch on(PD6). */
	GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 1);

	/* SPI SCLK pin (PB7). */
	GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0);
	/* SPI MOSI pin (PC0). */
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
	/* SPI MISO pin (PC1). */
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);

	/* SW2 pin (PA0). */
	GPIO_PinModeSet(gpioPortA, 0, gpioModeInputPullFilter, 1);
	GPIO_IntConfig(gpioPortA, 0, false, true, true );
	/* SW3 pin (PC14). */
	GPIO_PinModeSet(gpioPortC, 14, gpioModeInputPullFilter, 1);
	GPIO_IntConfig(gpioPortC, 14, false, true, true );
	/* SW4 pin (PD7). */
	GPIO_PinModeSet(gpioPortD, 7, gpioModeInputPullFilter, 1);
	GPIO_IntConfig(gpioPortD, 7, false, true, true );

}

void MAX35101_SetTime(void)
{
	char output[49];
	uint16_t year;
	uint8_t month, date, hour, minute;

	DOGM163_ClearChars(0, 0, 48);

	DOGM163_PrintChars(1, 0, 16, "SW2:ACCEPT");
	DOGM163_PrintChars(2, 0, 16, "SW3:UP  SW4:DOWN");

	// Set year.
	DOGM163_ClearChars(0, 0, 16);
	DOGM163_PrintChars(0, 0, 16, "SET YEAR:");

	MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
	year = POT_Data[POTCount].POT_TimeData.Time.tm_year + 1900;
	sprintf(output, "%04d",year);
  	DOGM163_PrintChars(0, 10, 4, output);

  	for(int loop = 0; loop < 10000000; loop++)
  	{
  		if(!GPIO_PinInGet(gpioPortA, 0))				// SW2 pushed down (low).
  		{
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortA, 0);

  			if(!GPIO_PinInGet(gpioPortA, 0))      		// SW2 pushed down (low) verified.
  			{
  				// write year/month to register.
  				MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
  				month = POT_Data[POTCount].POT_TimeData.Time.tm_mon;
  				MAX35101_Write_Register(MONTH_YEAR, ((((month/10)<<4) + month%10)<<8) + (((year-2000)/10)<<4) + (year-2000)%10);
  				break;
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortC, 14))  		// SW3 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortC, 14);

  			if(!GPIO_PinInGet(gpioPortC, 14))     		// SW3 pushed down (low) verified.
  			{
  				if(year < 2099)
  					year += 1;
  				sprintf(output, "%04d",year);
  				DOGM163_PrintChars(0, 10, 4, output);
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortD, 7))   		// SW4 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortD, 7);

  			if(!GPIO_PinInGet(gpioPortD, 7))      		// SW4 pushed down (low) verified.
  			{
  				if(year > 2000)
  					year -= 1;
  				sprintf(output, "%04d",year);
  				DOGM163_PrintChars(0, 10, 4, output);
  			}
  		}
  	} // Set year.

	// Set month.
	DOGM163_ClearChars(0, 0, 16);
	DOGM163_PrintChars(0, 0, 16, "SET MONTH:");

	MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
	month = POT_Data[POTCount].POT_TimeData.Time.tm_mon;
	sprintf(output, "%02d",month);
  	DOGM163_PrintChars(0, 11, 2, output);

  	for(int loop = 0; loop < 10000000; loop++)
  	{
  		if(!GPIO_PinInGet(gpioPortA, 0))				// SW2 pushed down (low).
  		{
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortA, 0);

  			if(!GPIO_PinInGet(gpioPortA, 0))      		// SW2 pushed down (low) verified.
  			{
  				// write year/month to register.
  				MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
  				year = POT_Data[POTCount].POT_TimeData.Time.tm_year + 1900;
  				MAX35101_Write_Register(MONTH_YEAR, ((((month/10)<<4) + month%10)<<8) + (((year-2000)/10)<<4) + (year-2000)%10);
  				break;
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortC, 14))  		// SW3 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortC, 14);

  			if(!GPIO_PinInGet(gpioPortC, 14))     		// SW3 pushed down (low) verified.
  			{
  				if(month < 12)
  					month += 1;
  				sprintf(output, "%02d",month);
  				DOGM163_PrintChars(0, 11, 2, output);
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortD, 7))   		// SW4 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortD, 7);

  			if(!GPIO_PinInGet(gpioPortD, 7))      		// SW4 pushed down (low) verified.
  			{
  				if(month > 1)
  					month -= 1;
  				sprintf(output, "%02d",month);
  				DOGM163_PrintChars(0, 11, 2, output);
  			}
  		}
  	} // Set month.

	// Set date.
	DOGM163_ClearChars(0, 0, 16);
	DOGM163_PrintChars(0, 0, 16, "SET DATE:");

	MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
	date = POT_Data[POTCount].POT_TimeData.Time.tm_mday;
	sprintf(output, "%02d",date);
  	DOGM163_PrintChars(0, 10, 2, output);

  	for(int loop = 0; loop < 10000000; loop++)
  	{
  		if(!GPIO_PinInGet(gpioPortA, 0))				// SW2 pushed down (low).
  		{
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortA, 0);

  			if(!GPIO_PinInGet(gpioPortA, 0))      		// SW2 pushed down (low) verified.
  			{
  				// write date to register.
  				MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
  				MAX35101_Write_Register(DAY_DATE, ((date/10)<<4) + date%10);
  				break;
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortC, 14))  		// SW3 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortC, 14);

  			if(!GPIO_PinInGet(gpioPortC, 14))     		// SW3 pushed down (low) verified.
  			{
  				if(date < 31)
  					date += 1;
  				sprintf(output, "%02d",date);
  				DOGM163_PrintChars(0, 10, 2, output);
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortD, 7))   		// SW4 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortD, 7);

  			if(!GPIO_PinInGet(gpioPortD, 7))      		// SW4 pushed down (low) verified.
  			{
  				if(date > 1)
  					date -= 1;
  				sprintf(output, "%02d",date);
  				DOGM163_PrintChars(0, 10, 2, output);
  			}
  		}
  	} // Set date.

	// Set hour.
	DOGM163_ClearChars(0, 0, 16);
	DOGM163_PrintChars(0, 0, 16, "SET HOUR:");

	MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
	hour = POT_Data[POTCount].POT_TimeData.Time.tm_hour;
	sprintf(output, "%02d",hour);
  	DOGM163_PrintChars(0, 10, 2, output);

  	for(int loop = 0; loop < 10000000; loop++)
  	{
  		if(!GPIO_PinInGet(gpioPortA, 0))				// SW2 pushed down (low).
  		{
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortA, 0);

  			if(!GPIO_PinInGet(gpioPortA, 0))      		// SW2 pushed down (low) verified.
  			{
  				// write hour/minute to register.
  				MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
  				minute = POT_Data[POTCount].POT_TimeData.Time.tm_min;
  				MAX35101_Write_Register(MINS_HRS, ((((minute/10)<<4) + minute%10)<<8) + ((hour/10)<<4) + hour%10);
  				break;
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortC, 14))  		// SW3 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortC, 14);

  			if(!GPIO_PinInGet(gpioPortC, 14))     		// SW3 pushed down (low) verified.
  			{
  				if(hour < 23)
  					hour += 1;
  				sprintf(output, "%02d",hour);
  				DOGM163_PrintChars(0, 10, 2, output);
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortD, 7))   		// SW4 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortD, 7);

  			if(!GPIO_PinInGet(gpioPortD, 7))      		// SW4 pushed down (low) verified.
  			{
  				if(hour > 0)
  					hour -= 1;
  				sprintf(output, "%02d",hour);
  				DOGM163_PrintChars(0, 10, 2, output);
  			}
  		}
  	} // Set hour.

	// Set minute.
	DOGM163_ClearChars(0, 0, 16);
	DOGM163_PrintChars(0, 0, 16, "SET MINUTE:");

	MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
	minute = POT_Data[POTCount].POT_TimeData.Time.tm_min;
	sprintf(output, "%02d",minute);
  	DOGM163_PrintChars(0, 12, 2, output);

  	for(int loop = 0; loop < 10000000; loop++)
  	{
  		if(!GPIO_PinInGet(gpioPortA, 0))				// SW2 pushed down (low).
  		{
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortA, 0);

  			if(!GPIO_PinInGet(gpioPortA, 0))      		// SW2 pushed down (low) verified.
  			{
  				// write hour/minute to register.
  				MAX35101_UpdateAndGetTime(&POT_Data[POTCount].POT_TimeData);
  				hour = POT_Data[POTCount].POT_TimeData.Time.tm_hour;
  				MAX35101_Write_Register(MINS_HRS, ((((minute/10)<<4) + minute%10)<<8) + ((hour/10)<<4) + hour%10);
  				break;
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortC, 14))  		// SW3 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortC, 14);

  			if(!GPIO_PinInGet(gpioPortC, 14))     		// SW3 pushed down (low) verified.
  			{
  				if(minute < 59)
  					minute += 1;
  				sprintf(output, "%02d",minute);
  				DOGM163_PrintChars(0, 12, 2, output);
  			}
  		}
  		else if(!GPIO_PinInGet(gpioPortD, 7))   		// SW4 pushed down (low).
  		{
  			loop = 0;
  			for(int loop1 = 0; loop1 < 300000; loop1++)	// Pin press debouncing.
  				GPIO_PinInGet(gpioPortD, 7);

  			if(!GPIO_PinInGet(gpioPortD, 7))      		// SW4 pushed down (low) verified.
  			{
  				if(minute > 0)
  					minute -= 1;
  				sprintf(output, "%02d",minute);
  				DOGM163_PrintChars(0, 12, 2, output);
  			}
  		}
  	} // Set minute.
}


