/**
  ******************************************************************************
  * @file    ControlGPIO.h
  * $Author: viewtool $
  * $Revision: 447 $
  * $Date:: 2020-06-02 18:24:57 +0800 #$
  * @brief   Ginkgo USB-GPIO适配器底层控制相关API函数定义.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROL1WIRE_H_
#define _CONTROL1WIRE_H_

#include <stdint.h>
#include "ErrorType.h"
#ifndef OS_UNIX
#include <Windows.h>
#else
#include <unistd.h>
#ifndef WINAPI
#define WINAPI
#endif
#endif

//The adapter type definition
#define VII_USB1WIRE			(1)		//Adapter Type: same as VII_USBI2C
//The adapter data initialization definition


//1Wire time parameter definition (ns)
typedef struct _VT_1WIRE_TIMING_CONFIG
{
    uint16_t tA;   //Timing for start signal Keeping
    uint16_t tB;   //Timing for start signal be established
    uint16_t tC;   //Timing for start signal be established
    uint16_t tD;   //Timing for start signal be established
    uint16_t tE;   //Timing for start signal be established
    uint16_t tF;   //Timing for start signal be established
    uint16_t tG;   //Timing for start signal be established
    uint16_t tH;   //Timing for start signal be established
    uint16_t tI;   //Timing for start signal be established
    uint16_t tJ;   //Timing for start signal be established
    uint16_t tR1;   //Timing for start signal be established
    uint16_t tR2;   //Timing for start signal be established
    uint16_t tR3;   //Timing for start signal be established
    uint16_t tR4;   //Timing for start signal be established
    uint16_t tR5;   //Timing for start signal be established
}VT_1WIRE_TIMING_CONFIG,*PVT_1WIRE_TIMING_CONFIG;

//1Wire Initialization
typedef struct _VT_1WIRE_INIT_CONFIG{
	/* for STM32F407, the GPIO port definition 
	   0: PORTA
	   1: PORTB
	   2: PORTC
	   3: PORTD
	   ...
	   8: PORTI (Maximum)
	*/
	uint8_t		port;	
	/* for STM32F407, the GPIO ping definition 
	   0: GPIO_PIN_0
	   1: GPIO_PIN_1
	   2: GPIO_PIN_2
	   3: GPIO_PIN_3
	   ...
	   15: GPIO_PIN_15 (Maximum)
	*/
	uint8_t		pin;	//Control mode: 0-Standard slave mode, 1-Standard mode, 2-GPIO mode
	VT_1WIRE_TIMING_CONFIG timing; 
}VT_1WIRE_INIT_CONFIG,*PVT_1WIRE_INIT_CONFIG;

#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI base_time_delay(int delay);
int32_t WINAPI delayus(int us_delay); 
int32_t WINAPI timer_count_cal(); 

int32_t WINAPI V1W_ScanDevice(uint8_t NeedInit=1);
int32_t WINAPI V1W_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);

int32_t WINAPI VT_OW_Init(int32_t DevType,int32_t DevIndex,VT_1WIRE_INIT_CONFIG *OWConfig);
int32_t WINAPI VT_OW_Set_Pin(int32_t DevType,int32_t DevIndex,unsigned char bt);
int32_t WINAPI VT_OW_Read_Pin(int32_t DevType,int32_t DevIndex, unsigned char *pData );
int32_t WINAPI VT_OW_Write_Bit(int32_t DevType,int32_t DevIndex,unsigned char bt);
int32_t WINAPI VT_OW_Write_Byte(int32_t DevType,int32_t DevIndex,unsigned char by);
int32_t WINAPI VT_OW_Read_Byte(int32_t DevType,int32_t DevIndex, unsigned char *pData);
int32_t WINAPI VT_OW_output_TX(int32_t DevType,int32_t DevIndex); 
int32_t WINAPI VT_OW_input_RX(int32_t DevType,int32_t DevIndex); 
int32_t WINAPI VT_OW_DS18B20_GetTempCelsiusByBinData(uint8_t _tempMSB, uint8_t _tempLSB, float *out); 

#ifdef __cplusplus
}
#endif

#endif

