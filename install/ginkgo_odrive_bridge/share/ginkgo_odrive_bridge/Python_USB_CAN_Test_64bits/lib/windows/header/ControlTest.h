/**
  ******************************************************************************
  * @file    ControlTest.h
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
#ifndef _CONTROLTEST_H_
#define _CONTROLTEST_H_

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

//适配器类型定义
#define VTT_USBTEST		(1)		//设备类型

typedef struct {
	uint8_t device; 
	uint8_t task; 
	int mode; 
}VTT_CONFIG; 

#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI VTT_ScanDevice(uint8_t NeedInit=1);
int32_t WINAPI VTT_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI VTT_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI VTT_TestInit(int32_t DevType, int32_t DevIndex, VTT_CONFIG *vtt_config);
int32_t WINAPI VTT_TestControl(int32_t DevType,int32_t DevIndex,uint8_t on);
int32_t WINAPI VTT_TestReadDataSize(int32_t DevType,int32_t DevIndex);
int32_t WINAPI VTT_TestReadData(int32_t DevType,int32_t DevIndex,uint8_t *data, int32_t size);
int32_t WINAPI VTT_GetVersionTime(int32_t DevType,int32_t DevIndex,uint8_t *dt, int32_t *dt_size,uint8_t *tm, int32_t *tm_size);
#ifdef __cplusplus
}
#endif


#endif
