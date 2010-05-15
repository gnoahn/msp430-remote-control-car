/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2008 Freescale Semiconductor, Inc.
* (c) Copyright 2001-2004 Motorola, Inc.
* ALL RIGHTS RESERVED.
*
* =================================================================== *
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY              *
* EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  *
* PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL FREESCALE OR             *
* ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT        *
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;        *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)            *
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, *
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       *
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED *
* OF THE POSSIBILITY OF SUCH DAMAGE.                                  *
* =================================================================== *
*
***************************************************************************//*!
*
* @file      sw_iic.h
*
* @author    R20246
* 
* @version   2.0.0.1
* 
* @date      FEB-14-2008
* 
* @brief     ION (MMA7455L) - Ion MMA7455L in Measure mode (using SPI or IIC) 
*
*******************************************************************************
*
* 
*
******************************************************************************/
#ifndef SW_IIC_H
#define SW_IIC_H

#include "derivative.h"

#define ION_ADDR 0x1D /* ION address is 1D */

#define SCL     PTCD_PTCD3        
#define SCL_D   PTCDD_PTCDD3  
#define SCL_PU  PTCPE_PTCPE3  
#define SDA     PTCD_PTCD2        
#define SDA_D   PTCDD_PTCDD2  
#define SDA_PU  PTCPE_PTCPE2 
#define SS      PTBD_PTBD3       
#define SS_D    PTBDD_PTBDD3       
#define SS_PU   PTBPE_PTBPE3      

#define P_OUT   1
#define P_IN    0
#define PU_ON   1
#define PU_OFF  0

#define ACK 0
#define NACK 1

#define IIC_ACK  0
#define IIC_NACK 1

#define START_DELAY 1
#define ADDRESS_DELAY 1

#define SDA_HOLD 5          //20MHz Bus Clock: 1  => IIC 110kHz
#define SCL_HOLD 5          //                 5  => IIC  70kHz

#define MAX_IIC_TIMEOUT 0x9F00

// FUNCTION DECLARATION
void SwIIC_Delay(byte); 
void SwIIC_Wait(byte);
void IIC_DATA_0(void);
void IIC_DATA_1(void);
byte IIC_GetBit(void);
void IIC_START(void);
void IIC_REPEAT_START(void);
void IIC_STOP(void);
byte IIC_ReadByte(byte ack); 
void IIC_SendByte(byte tx_byte);
byte IIC_SendByteM(byte tx_byte);
void SwIIC_Init(void); 
void SwIIC_SendAddressWrite(byte tx_byte);
void SwIIC_SendAddressRead(byte tx_byte);
void SwIIC_SendRegister(byte tx_byte);
byte SwIIC_ReadByte(byte RegAdd);
void SwIIC_WriteByte(byte RegAdd, byte RegVal);
void SwIIC_MultiReadBytes(byte num, byte *buff, byte cnt);
void SwIIC_MultiWriteBytes(byte num, byte *buff, byte cnt);
  
#endif //SW_IIC_H