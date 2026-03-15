/**
  ******************************************************************************
  * @file    ControlI2C.h
  * $Author: viewtool $
  * $Revision: 447 $
  * $Date:: 2020-06-02 18:24:57 +0800 #$
  * @brief   Ginkgo USB-I2C Adapter API function definition.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROLI2C_H_
#define _CONTROLI2C_H_

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
#define VII_USBI2C			(1)		//Adapter Type
//The adapter data initialization definition
#define VII_ADDR_7BIT		(7)		//7-bit address mode
#define VII_ADDR_10BIT		(10)	//10-bit address mode
#define VII_HCTL_SLAVE_MODE	(0)		//Standard slave mode
#define VII_HCTL_MODE		(1)		//Standard mode
#define VII_SCTL_MODE		(2)		//GPIO mode
#define VII_MASTER			(1)		//Master
#define VII_SLAVE			(0)		//Slave
#define VII_SUB_ADDR_NONE	(0)		//No sub-address
#define VII_SUB_ADDR_1BYTE	(1)		//1Byte sub-address
#define VII_SUB_ADDR_2BYTE	(2)		//2Byte sub-address
#define VII_SUB_ADDR_3BYTE	(3)		//3Byte sub-address
#define VII_SUB_ADDR_4BYTE	(4)		//4Byte sub-address

#define VII_EVENT_PULLUP_FALLING			0x000C
#define VII_EVENT_PULLUP_RISING				0x0008
#define VII_EVENT_PULLUP_RISING_FALLING		0x0010
#define VII_EVENT_FLOAT_FALLING				0x030C
#define VII_EVENT_FLOAT_RISING				0x0308
#define VII_EVENT_FLOAT_RISING_FALLING		0x0310

// Event Pin define
#define EVENT_PIN0         (1<<0)    //GPIO_0
#define EVENT_PIN1         (1<<1)    //GPIO_1
#define EVENT_PIN2         (1<<2)    //GPIO_2
#define EVENT_PIN3         (1<<3)    //GPIO_3
#define EVENT_PIN4         (1<<4)    //GPIO_4
#define EVENT_PIN5         (1<<5)    //GPIO_5
#define EVENT_PIN6         (1<<6)    //GPIO_6
#define EVENT_PIN7         (1<<7)    //GPIO_7
#define EVENT_PIN8         (1<<8)    //GPIO_8
#define EVENT_PIN9         (1<<9)    //GPIO_9
#define EVENT_PIN10        (1<<10)    //GPIO_10
#define EVENT_PIN11        (1<<11)    //GPIO_11
#define EVENT_PIN12        (1<<12)    //GPIO_12
#define EVENT_PIN13        (1<<13)    //GPIO_13
#define EVENT_PIN14        (1<<14)    //GPIO_14
#define EVENT_PIN15        (1<<15)    //GPIO_15
//1.Data types of Ginkgo series adapter information.
#define PRODUCT_NAME_SIZE     64
#define HARDWARE_VERSION_SIZE     4
#define FIRMWARE_VERSION_SIZE     4
#define SERIAL_NUMBER_SIZE     12
#define DLL_VERSION_SIZE       12 

typedef  struct  _VII_BOARD_INFO{
	uint8_t		ProductName[PRODUCT_NAME_SIZE];	//硬件名称，比如“Ginkgo-SPI-Adaptor”（注意：包括字符串结束符‘\0’）
	uint8_t		FirmwareVersion[FIRMWARE_VERSION_SIZE];	//固件版本
	uint8_t		HardwareVersion[HARDWARE_VERSION_SIZE];	//硬件版本
	uint8_t		SerialNumber[SERIAL_NUMBER_SIZE];	//适配器序列号
	uint8_t     DllVersion[DLL_VERSION_SIZE]; 
} VII_BOARD_INFO,*PVII_BOARD_INFO; 


//2.I2C data types initialization Define
typedef struct _VII_INIT_CONFIG{
	uint8_t		MasterMode;		//Master-slave choice: 0-slave, 1-master
	uint8_t		ControlMode;	//Control mode: 0-Standard slave mode, 1-Standard mode, 2-GPIO mode
	uint8_t		AddrType;		//7-7bit mode, 10-10bit mode
	uint8_t		SubAddrWidth;	//Sub-Address width: value of 0~4, 0 means no Sub-Address mode
	uint16_t	Addr;			//Device address in salve mode
	uint32_t	ClockSpeed;		//Clock frequency(HZ)
}VII_INIT_CONFIG,*PVII_INIT_CONFIG;

//3.I2C time parameter definition in GPIO mode(ms)
typedef struct _VII_TIME_CONFIG
{
    uint16_t tHD_STA;   //Timing for start signal Keeping
    uint16_t tSU_STA;   //Timing for start signal be established
    uint16_t tLOW;      //Timing for clock low level
    uint16_t tHIGH;     //Timing for clock high level
    uint16_t tSU_DAT;   //Timing for data input be established
    uint16_t tSU_STO;   //Timing for stop signal be established
    uint16_t tDH;       //Timing for data output Keeping
    uint16_t tDH_DAT;   //Timing for data input Keeping
    uint16_t tAA;       //SCL lower to SDA output and response signal
    uint16_t tR;        //Timing for SDA and SCL rising
    uint16_t tF;        //Timing for SDA and SCL going down
    uint16_t tBuf;      //Free timing of the bus until the new mission
    uint16_t tACK[4];
    uint16_t tStart;
    uint16_t tStop;
}VII_TIME_CONFIG,*PVII_TIME_CONFIG;

#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI VII_ScanDevice(uint8_t NeedInit=1);
int32_t WINAPI VII_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI VII_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI VII_ReadBoardInfo(int32_t DevIndex,PVII_BOARD_INFO pInfo);
int32_t WINAPI VII_TimeConfig(int32_t DevType, int32_t DevIndex, int32_t I2CIndex, PVII_TIME_CONFIG pTimeConfig);
int32_t WINAPI VII_InitI2C(int32_t DevType, int32_t DevIndex, int32_t I2CIndex, PVII_INIT_CONFIG pInitConfig);
int32_t WINAPI VII_PullupConfig(int32_t DevType, int32_t DevIndex, int32_t I2CIndex, uint32_t pullup);
int32_t WINAPI VII_WriteBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t SlaveAddr,uint32_t SubAddr,uint8_t *pWriteData,uint16_t Len);
int32_t WINAPI VII_WriteReadBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t SlaveAddr,uint32_t WriteSubAddr,uint8_t *pWriteData,uint16_t WriteLen,uint32_t ReadSubAddr,uint8_t *pReadData,uint16_t ReadLen,uint16_t IntervalTime);
int32_t WINAPI VII_ReadBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t SlaveAddr,uint32_t SubAddr,uint8_t *pReadData,uint16_t Len);
int32_t WINAPI VII_SetUserKey(int32_t DevType,int32_t DevIndex,uint8_t* pUserKey);
int32_t WINAPI VII_CheckUserKey(int32_t DevType,int32_t DevIndex,uint8_t* pUserKey);
int32_t WINAPI VII_SlaveReadBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint8_t* pReadData,uint16_t *pLen);
int32_t WINAPI VII_SlaveWriteBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint8_t* pWriteData,uint16_t Len);
int32_t WINAPI VII_SlaveWriteRemain(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t* pRemainBytes);
int32_t WINAPI VII_PinEventReadBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t EventPin,uint16_t EventType,uint16_t SlaveAddr,uint32_t SubAddr,uint8_t* pReadData,uint16_t Len,int32_t TimeOutMs);
int32_t WINAPI VII_PinEventWriteBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t EventPin,uint16_t EventType,uint16_t SlaveAddr,uint32_t SubAddr,uint8_t *pWriteData,uint16_t Len,int32_t TimeOutMs);
int32_t WINAPI VII_WriteUserData(int32_t DevType,int32_t DevIndex,int32_t OffsetAddr,uint8_t *pData,int32_t DataNum);
int32_t WINAPI VII_ReadUserData(int32_t DevType,int32_t DevIndex,int32_t OffsetAddr,uint8_t *pData,int32_t DataNum);
int32_t WINAPI VII_Slave24C08WriteBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t DeviceAddr,uint32_t SubAddr,uint8_t* pWriteData,uint16_t Len); 
int32_t WINAPI VII_Slave24C08ReadBytes(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t DeviceAddr,uint32_t SubAddr,uint8_t* pReadData,uint16_t *pLen); 
int32_t WINAPI VII_SlaveReadBytesEx(int32_t DevType,int32_t DevIndex,int32_t I2CIndex,uint16_t DeviceAddr,uint32_t SubAddr, uint8_t* pReadData,uint16_t *pLen); 

#ifdef __cplusplus
}
#endif


#endif

