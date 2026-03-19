/**
  ******************************************************************************
  * @file    ControlSDI.h
  * $Author: ViewTool  $
  * $Revision: 447 $
  * $Date:: 2013-06-29 18:24:57 +0800 #$
  * @brief   Ginkgo USB-SDI12适配器底层控制相关API函数定义.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROLSDI_H_
#define _CONTROLSDI_H_

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
#define SDI_USBSDI			(1)		//设备类型
#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI SDI_ScanDevice(uint8_t NeedInit=1);
int32_t WINAPI SDI_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI SDI_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI SDI_InitSDI(int32_t DevType, int32_t DevIndex, uint8_t SDIIndex);
int32_t WINAPI SDI_SendCmd(int32_t DevType, int32_t DevIndex, uint8_t SDIIndex,uint8_t *pCommand,uint8_t *pRespond,uint8_t CheckCRC);

#ifdef __cplusplus
}
#endif

#endif