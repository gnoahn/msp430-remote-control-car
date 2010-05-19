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
* @file      sw_iic.c
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

#include "sw_iic.h"

//----I2C Variables -------------------------------------------------------
byte ak1=NACK;
byte ak2=NACK;
byte ak3=NACK;
//-------------------------------------------------------------------------

/**************************************************************
*	Function: 	Delay
*	Parameters: 16-bit delay value
*	Return:		none
*
*	Simple delay loop.
**************************************************************/ 
void SwIIC_Delay(byte count) 
{
	byte i;
	for (i=0; i<count; i++)
	{
    //asm nop;           
	}
}

/**************************************************************
*	Function: 	Wait
*	Parameters: 8-bit delay value
*	Return:		none
*
*	Simple delay loop. Little longer than delay function.
**************************************************************/
void SwIIC_Wait(byte loop)
{
 while(--loop); 
 {                     
    //asm nop;      
 }
}

/**************************************************************
*	Function: 	SwIIC_Init
*	Parameters: none
*	Return:		none
*
*	Function initialize sw IIC interface
**************************************************************/
void SwIIC_Init(void)
{
  SCL_D = P_IN;
  SDA_D = P_IN;
  SS_D = P_OUT; 
  SCL_PU = PU_ON;
  SDA_PU = PU_ON;
  SS=1;                 
}

/**************************************************************
*	Function: 	IIC_DATA_0                 
*	Parameters: none
*	Return:		none
*
*	Physical layer of IIC - Write '0' on bus   
**************************************************************/
void IIC_DATA_0(void)
{
  word i = 0; 
  
  SDA = 0; 
  SDA_D = P_OUT;
  SwIIC_Wait(SDA_HOLD); 
  SCL_D = P_IN; 
  while((!SCL) && (i++ < MAX_IIC_TIMEOUT));
  SwIIC_Wait(SCL_HOLD); 
  SCL = 0; 
  SCL_D = P_OUT;
  SwIIC_Wait(SCL_HOLD);
}

/**************************************************************
*	Function: 	IIC_DATA_1               
*	Parameters: none
*	Return:		none
*
*	Physical layer of IIC - Write '1' on bus   
**************************************************************/
void IIC_DATA_1(void)
{
  word i = 0;
  
  SDA_D = P_IN;
  SwIIC_Wait(SDA_HOLD);
  SCL_D = P_IN; 
  while((!SCL) && (i++ < MAX_IIC_TIMEOUT));
  SwIIC_Wait(SCL_HOLD); 
  SCL = 0; 
  SCL_D = P_OUT;
  SwIIC_Wait(SCL_HOLD);
}

/**************************************************************
*	Function: 	IIC_GetBit                       
*	Parameters: none
*	Return:		value of bit on bus
*
*	Physical layer of IIC - Read a bit from bus IIC 
**************************************************************/
byte IIC_GetBit(void)
{
  byte level;
  word i = 0;

  SDA_D = P_IN;
  SwIIC_Wait(SDA_HOLD);
  SCL_D = P_IN; 
  while((!SCL) && (i++ < MAX_IIC_TIMEOUT));
  SwIIC_Wait(SCL_HOLD);
  level = SDA;
  SCL = 0; 
  SCL_D = P_OUT;
  SwIIC_Wait(SCL_HOLD);
  return level;  
} 

/**************************************************************
*	Function: 	IIC_START                   
*	Parameters: none
*	Return:		none
*
*	Physical layer of IIC - Set up START condition on bus
**************************************************************/
void IIC_START(void)
{
  SDA = 0; 
  SDA_D = P_OUT;
  SwIIC_Wait(SDA_HOLD);
  SCL = 0; 
  SCL_D = P_OUT;
  SwIIC_Wait(SCL_HOLD);
}

/**************************************************************
*	Function: 	IIC_REPEAT_START              
*	Parameters: none
*	Return:		none
*
*	Physical layer of IIC - Set up REPEAT_START condition on bus
**************************************************************/
void IIC_REPEAT_START(void)
{
  word i = 0;
  
  SDA_D = P_IN;
  SwIIC_Wait(SDA_HOLD);
  SCL_D = P_IN; 
  while((!SCL) && (i++ < MAX_IIC_TIMEOUT));
  SwIIC_Wait(SCL_HOLD); 
  IIC_START();
}

/**************************************************************
*	Function: 	IIC_STOP                       
*	Parameters: none
*	Return:		none
*
*	Physical layer of IIC - Set up STOP condition on bus
**************************************************************/
void IIC_STOP(void)
{
  word i = 0;
  
  SDA = 0; 
  SDA_D = P_OUT;
  SwIIC_Wait(SDA_HOLD); 
  SCL_D = P_IN; 
  while((!SCL) && (i++ < MAX_IIC_TIMEOUT));
  SwIIC_Wait(SCL_HOLD); 
  SDA_D = P_IN;
  SwIIC_Wait(SDA_HOLD);
}

/**************************************************************
*	Function: 	SwIIC_SendAddressWrite          
*	Parameters: tx_byte
*	Return:		none
*
*	Send device address on bus with write flag 
**************************************************************/
void SwIIC_SendAddressWrite(byte tx_byte)
{
  byte k;
 
  tx_byte <<= 1;    
  tx_byte &= 0xfe;  
 
  for(k=8;k>0;k--)    
  {                                
    if (tx_byte & 0x80)            
      IIC_DATA_1();                
    else 
      IIC_DATA_0();
    tx_byte <<= 1;
  } 
  ak1=IIC_GetBit(); 
}
   

/**************************************************************
*	Function: 	SwIIC_SendAddressRead           
*	Parameters: tx_byte
*	Return:		none
*
*	Send device address on bus with read flag 
**************************************************************/
void SwIIC_SendAddressRead(byte tx_byte)
{
  byte k;           
 
  tx_byte <<= 1;      
  tx_byte |= 0x01;    
                                           
  for(k=8;k>0;k--)    
  {                                                                    
    if (tx_byte & 0x80)                                                
      IIC_DATA_1();
    else 
      IIC_DATA_0();
    tx_byte <<= 1;
  }
  ak3=IIC_GetBit(); 
}
   
/**************************************************************
*	Function: 	SwIIC_SendRegister               
*	Parameters: tx_byte
*	Return:		none
*
*	Send device internal register address on bus  
**************************************************************/   
void SwIIC_SendRegister(byte tx_byte)
{
  byte k;
      
  for(k=8;k>0;k--)    
  {         
    if (tx_byte & 0x80) 
      IIC_DATA_1();
    else 
      IIC_DATA_0();
    tx_byte <<= 1;
  }
  ak2=IIC_GetBit(); 
}

/**************************************************************
*	Function: 	IIC_ReadByte                    
*	Parameters: ack
*	Return:		val
*
*	Read a byte from IIC bus  
**************************************************************/
byte IIC_ReadByte(byte ack)      
{                                  
byte i, val;

  for(i=8; i>0; i--)
  {
    val <<= 1;
    val |= IIC_GetBit();
  }
  
  if (ack == IIC_ACK)            
    IIC_DATA_0();                
  else
    IIC_DATA_1();                
                                                                      
  return val;  
}

/**************************************************************
*	Function: 	IIC_SendByte                     
*	Parameters: tx_byte
*	Return:		none
*
*	Write a byte to IIC bus  
**************************************************************/
void IIC_SendByte(byte tx_byte)
{
  byte k;      
 
  for(k=8;k>0;k--)    
  {         
    if (tx_byte & 0x80) 
      IIC_DATA_1();
    else 
      IIC_DATA_0();
    tx_byte <<= 1;
  }
  ak3=IIC_GetBit(); 
}

byte IIC_SendByteM(byte tx_byte)
{
  byte k;      
 
  for(k=8;k>0;k--)    
  {         
    if (tx_byte & 0x80) 
      IIC_DATA_1();
    else 
      IIC_DATA_0();
    tx_byte <<= 1;
  }
  
  ak3=IIC_GetBit(); 
  
  return(ak3);
}

/**************************************************************
*	Function: 	SwIIC_ReadByte                   
*	Parameters: RegAdd
*	Return:		none
*
*	Complete data transfer - read a byte from device register 
* Note: Secured routine through check of acknowledge
**************************************************************/
byte SwIIC_ReadByte(byte RegAdd)                        
{  
      byte temp;
      ak1=NACK;
      ak2=NACK;
      ak3=NACK;
     
      while(!(ak3==ACK))
      {   
        IIC_START();
        SwIIC_Delay(START_DELAY);                 
        SwIIC_SendAddressWrite(ION_ADDR); 
        SwIIC_Delay(ADDRESS_DELAY);  
          if(!ak1)                                   
          { 
          SwIIC_SendRegister(RegAdd); 
              if(!ak2)                               
              {      
              IIC_REPEAT_START(); 
              SwIIC_Delay(START_DELAY);           
              SwIIC_SendAddressRead(ION_ADDR);
              if(!ak3) temp=IIC_ReadByte(IIC_NACK);  
              }
          }
      IIC_STOP();
      ak1=NACK;
      ak2=NACK;
      }  
    return(temp);    
}

/**************************************************************
*	Function: 	SwIIC_WriteByte                  
*	Parameters: RegAdd, RegVal
*	Return:		none
*
*	Complete data transfer - write a byte to device (ION) register
* Note: Secured routine through check of acknowledge 
**************************************************************/
void SwIIC_WriteByte(byte RegAdd, byte RegVal)                      
{     
              
      ak1=NACK;
      ak2=NACK;
      ak3=NACK;
      
      while(ak3)
      { 
        IIC_START();
        SwIIC_Delay(START_DELAY);  
        SwIIC_SendAddressWrite(ION_ADDR); 
        SwIIC_Delay(ADDRESS_DELAY);   
          if(!ak1)                                 
          {
              SwIIC_SendRegister(RegAdd); 
              SwIIC_Delay(ADDRESS_DELAY);
              if(!ak2)                             
              {
              IIC_SendByte(RegVal);
              }
          }
        IIC_STOP();
        ak1=NACK;
        ak2=NACK;
      }           
}

/**************************************************************
*	Function: 	SwIIC_MultiRead                  
*	Parameters: RegAdd, *buff (data), cnt - length
*	Return:		none
*
*	Complete data transfer - multiple write a bytes to device (ION) registers 
* Note: Secured routine through check of acknowledge
**************************************************************/
void SwIIC_MultiReadBytes(byte RegAdd, byte *buff, byte cnt)        
{  
  byte i;
  
  i=0; 
  ak1=NACK;
  ak2=NACK; 
  ak3=NACK;
    
      while(!(ak3==ACK))
      {          
       IIC_START();
       SwIIC_Delay(START_DELAY);                 
       SwIIC_SendAddressWrite(ION_ADDR); 
       SwIIC_Delay(ADDRESS_DELAY);
       if(!ak1) 
       { 
        SwIIC_SendRegister(RegAdd);
        if(!ak2)      
        {
         IIC_REPEAT_START();
         SwIIC_SendAddressRead(ION_ADDR);
         SwIIC_Delay(ADDRESS_DELAY);
         if(!ak3)
         {
          for(i=0;i<(cnt - 1);i++)
          {
           *buff++ = IIC_ReadByte(IIC_ACK);
          }
          *buff++ = IIC_ReadByte(IIC_NACK);
         }
        }
       }
       IIC_STOP();
       ak1=NACK;
       ak2=NACK; 
      }
}

/**************************************************************
*	Function: 	SwIIC_MultiWrite                
*	Parameters: RegAdd, *buff (data), cnt - length
*	Return:		none
*
*	Complete data transfer - multiple write a bytes into device (ION) registers 
* Note: Secured routine through check of acknowledge
**************************************************************/
void SwIIC_MultiWriteBytes(byte RegAdd, byte *buff, byte cnt) 
{  
  byte i,j,ak[32];
  
  i=0; 
  ak1=NACK;
  ak2=NACK; 
  ak3=NACK;
  for(j=0;j<32;j++) ak[j]=NACK;
    
      while(!(ak[cnt+2]==ACK))
      {                                                   
        IIC_START();
        SwIIC_Delay(START_DELAY);                       
        SwIIC_SendAddressWrite(ION_ADDR); 
        SwIIC_Delay(ADDRESS_DELAY);
          if(!ak1)                                 
          {              
              SwIIC_SendRegister(RegAdd);      
              SwIIC_Delay(ADDRESS_DELAY); 
              ak[2]=ak2; 
              while((!ak[i+2])&&(i<cnt))      
              {
                ak[i+3]=IIC_SendByteM(*(buff++));
                i++;
              } 
          }
        IIC_STOP();
        ak1=NACK;
        for (j=0;j<(cnt-1);j++) ak[j+2]=NACK;
      }
}