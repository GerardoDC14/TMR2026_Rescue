/**
  ******************************************************************************
  * @file    ControlPWM.h
  * $Author: viewtool $
  * $Revision: 447 $
  * $Date:: 2020-06-02 18:24:57 +0800 #$
  * @brief   Ginkgo USB-PWM适配器底层控制相关API函数定义.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROLPWM_H_
#define _CONTROLPWM_H_

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
#define VPI_USBBOOT		(2)

#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI VBI_ScanDevice(uint8_t NeedInit=1);
int32_t WINAPI VBI_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI VBI_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI VBI_Boot(int32_t DevType, int32_t DevIndex, int mode); 
int32_t WINAPI VBI_Reset(int32_t DevType, int32_t DevIndex, int mode); 

#ifdef __cplusplus
}
#endif

#endif
