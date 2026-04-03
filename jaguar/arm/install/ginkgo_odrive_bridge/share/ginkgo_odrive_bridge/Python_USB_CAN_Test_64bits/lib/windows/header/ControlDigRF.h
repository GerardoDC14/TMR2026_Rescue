/**
  ******************************************************************************
  * @file    ControlDigRF.h
  * $Author: viewtool $
  * $Revision: 447 $
  * $Date:: 2020-06-02 18:24:57 +0800 #$
  * @brief   Ginkgo USB-DigRF Adapter API function definition.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROLDIGRF_H_
#define _CONTROLDIGRF_H_

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
#define VDI_USBDIGRF		(1)		//Adapter Type
//Channel define
#define	VDI_CH0		(1<<0)	//CH0
#define	VDI_CH1		(1<<1)	//CH1
#define	VDI_CH2		(1<<2)	//CH2
#define	VDI_CH3		(1<<3)	//CH3

#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI VDI_ScanDevice(uint8_t NeedInit);
int32_t WINAPI VDI_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI VDI_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI VDI_InitDigRF(int32_t DevType,int32_t DevIndex,uint8_t Channel,uint16_t Speed,uint8_t AddrBytes);
int32_t WINAPI VDI_WriteBytes(int32_t DevType,int32_t DevIndex,uint8_t Channel,uint32_t Addr,uint8_t *pWriteData,uint8_t WriteLen,uint16_t PreClkCnt,uint16_t AftClkCnt);
int32_t WINAPI VDI_ReadBytes(int32_t DevType,int32_t DevIndex,uint8_t Channel,uint32_t Addr,uint8_t *pReadData,uint8_t ReadLen,uint16_t PreClkCnt,uint16_t AftClkCnt);
int32_t WINAPI VDI_ReadMemory(int32_t DevType,int32_t DevIndex,uint8_t Channel,uint32_t Addr,uint8_t *pReadData,uint8_t ReadLen,uint16_t PreClkCnt,uint16_t AftClkCnt);
int32_t WINAPI VDI_ReadRegister(int32_t DevType,int32_t DevIndex,uint8_t Channel,uint32_t Addr,uint8_t *pReadData,uint8_t ReadLen,uint16_t PreClkCnt,uint16_t AftClkCnt);
#ifdef __cplusplus
}
#endif


#endif

