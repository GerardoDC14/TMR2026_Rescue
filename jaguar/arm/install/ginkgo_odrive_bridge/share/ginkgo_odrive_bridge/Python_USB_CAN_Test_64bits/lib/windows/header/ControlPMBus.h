/**
  ******************************************************************************
  * @file    ControlPMBus.h
  * $Author: viewtool $
  * $Revision: 447 $
  * $Date:: 2020-06-02 18:24:57 +0800 #$
  * @brief   Ginkgo USB-PMBus适配器底层控制相关API函数定义.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#ifndef _CONTROLPMBUS_H_
#define _CONTROLPMBUS_H_

#include <stdint.h>
#include "ErrorType.h"
#ifndef OS_UNIX
#include <Windows.h>
#endif

//适配器类型定义
#define VII_USBI2C			(1)		//设备类型

//SMBUS总线错误码
#define PMBUS_OK                        0x00    ///< Packet was sent correctly
#define PMBUS_ERROR_SLAVE_NOT_SUPPORTED 0x01    ///< Slave mode is not supported 
#define PMBUS_ERROR_BUSOFF              0x02
#define PMBUS_ERROR_TXFULL              0x03
#define PMBUS_ERROR_BUSY                0x04
#define PMBUS_ERROR_RXEMPTY             0x05
#define PMBUS_ERROR_OVERRUN             0x06
#define PMBUS_ERROR_TIMEOUT             0x07    ///< Timeout occured during sending the packet 
#define PMBUS_ERROR_INVALID_SIZE        0x08    ///< Invalid size of received packet
#define PMBUS_ERROR_PACKET_TOO_LONG     0x09    ///< Packet to sent does not fit into internal buffer
#define PMBUS_ERROR_PARAMETER           0x0A    ///< Invalid parameter
#define PMBUS_ERROR_PEC                 0x0B    ///< PEC error
#define PMBUS_ERROR_NACK				0x0C	///< NACK error
#define PMBUS_ERROR_ARLO				0x0D	///< Arbitration lost (master mode)


#ifdef __cplusplus
extern "C"
{
#endif

int32_t WINAPI PMBus_ScanDevice(uint8_t NeedInit);
int32_t WINAPI PMBus_OpenDevice(int32_t DevType,int32_t DevIndex,int32_t Reserved);
int32_t WINAPI PMBus_CloseDevice(int32_t DevType,int32_t DevIndex);
int32_t WINAPI PMBus_HardInit(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint32_t ClockSpeed, uint8_t OwnAddr);
int32_t WINAPI PMBus_WriteByte(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint8_t Data,uint8_t PEC);
int32_t WINAPI PMBus_ReadByte(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode, uint8_t *pData,uint8_t PEC);
int32_t WINAPI PMBus_SendByte(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint8_t PEC);
int32_t WINAPI PMBus_WriteWord(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint16_t Data,uint8_t PEC);
int32_t WINAPI PMBus_ReadWord(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode, uint16_t *pData,uint8_t PEC);
int32_t WINAPI PMBus_WriteByteExt(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCodeExt,uint8_t CommandCode,uint8_t Data,uint8_t PEC);
int32_t WINAPI PMBus_ReadByteExt(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCodeExt,uint8_t CommandCode, uint8_t *pData,uint8_t PEC);
int32_t WINAPI PMBus_WriteWordExt(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCodeExt,uint8_t CommandCode,uint16_t Data,uint8_t PEC);
int32_t WINAPI PMBus_ReadWordExt(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCodeExt,uint8_t CommandCode, uint16_t *pData,uint8_t PEC);
int32_t WINAPI PMBus_BlockWrite(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint8_t *pData,uint8_t ByteCount,uint8_t PEC);
int32_t WINAPI PMBus_BlockRead(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint8_t *pData,uint8_t *pByteCount,uint8_t PEC);
int32_t WINAPI PMBus_BlockProcessCall(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t SlaveAddr,uint8_t CommandCode,uint8_t *pWriteData,uint8_t WriteByteCount,uint8_t *pReadData,uint8_t *pReadByteCount,uint8_t PEC);
int32_t WINAPI PMBus_GroupCmd(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t *pGroupCmdData,uint8_t CmdNum,uint8_t PEC);
int32_t WINAPI PMBus_GetAlert(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t *pAlertFlag);
int32_t WINAPI PMBus_SetControl(int32_t DevType,int32_t DevIndex,int32_t PMBusIndex,uint8_t Value);

#ifdef __cplusplus
}
#endif


#endif

