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
* @file      sw_spi.c
*
* @author    R20246
* 
* @version   1.0.0.0
* 
* @date      MAR-6-2008
* 
* @brief     ION (MMA7455L)
*                           Ion MMA7455L using SPI or IIC Communication Protocol 
*                           in Measure mode, Level and Pulse Detection   
*
*******************************************************************************
*
* 
*
******************************************************************************/

#include "sw_spi.h"

byte rx_byte = 0;

/**************************************************************
*	Function: 	Wait
*	Parameters: 8-bit delay value
*	Return:		none
*
*	Simple delay loop. Little longer than delay function.
**************************************************************/
void SwSPI_Wait(byte loop)
{
 while(--loop); 
 {                     
    //asm nop;      
 }
}

/**************************************************************
*	Function: 	SwSPI_Init
*	Parameters: none
*	Return:		none
*
*	Function initialize sw SPI interface
**************************************************************/
void SwSPI_Init(void)
{
  INIT_SPI_PINS_MACRO  
}

/**************************************************************
*	Function: 	SwSPI_SendChar
*	Parameters: tx_byte
*	Return:		none
*
*	Function send by SPI a byte
**************************************************************/
void SwSPI_SendChar(byte tx_byte)
{
  byte i;
  
  rx_byte = 0;  
  for(i=0;i<8;i++)
  {    
    MOSI = (0x80 & tx_byte)? 0x01 : 0x00 ;
    SwSPI_Wait(MOSI_HOLD);
    SCLK = 1;
    SwSPI_Wait(SCLK_HOLD);
    tx_byte <<= 1;   
    if(MISO) rx_byte |= 1 << (7 - i);
    SCLK = 0;
  }
}

/**************************************************************
*	Function: 	SwSPI_ReceiveChar
*	Parameters: none
*	Return:		receive byte
*
*	Function return back a byte that was read on last SPI transfer
**************************************************************/
byte SwSPI_ReceiveChar(void)
{
 return rx_byte;
}

/**************************************************************
*	Function: 	SwSPI_ReadByte
*	Parameters: RegAdd - number of register
*	Return:		receive byte
*
*	Function read one byte from ION sensor by SPI
**************************************************************/
byte SwSPI_ReadByte(byte RegAdd)
{
  byte i;
  
  i = (RegAdd <<1) & 0x7f;
  SwSPI_Start();
  SwSPI_SendChar(i);
  SwSPI_SendChar(0);
  SwSPI_Stop();
  return (SwSPI_ReceiveChar());  
}

/**************************************************************
*	Function: 	SwSPI_WriteByte
*	Parameters: RegAdd - number of register, val - writing value
*	Return:		none
*
*	Function write one byte to ION sensor
**************************************************************/
void SwSPI_WriteByte(byte RegAdd, byte val)
{
  byte i;

  i = (RegAdd <<1) | 0x80;
  SwSPI_Start();
  SwSPI_SendChar(i);
  SwSPI_SendChar(val);
  SwSPI_Stop();  
}