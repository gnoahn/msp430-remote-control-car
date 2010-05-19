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
* @file      sw_spi.h
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

#ifndef SW_SPI_H
#define SW_SPI_H

#include "derivative.h"

// Pins definition
#define SCLK    PTBD_PTBD7  
#define SCLK_D  PTBDD_PTBDD7   
#define MOSI    PTBD_PTBD6     
#define MOSI_D  PTBDD_PTBDD6
#define MISO    PTGD_PTGD0    
#define MISO_D  PTGDD_PTGDD0 
#define MISO_PU PTGPE_PTGPE0 
#define SS      PTGD_PTGD1    
#define SS_D    PTGDD_PTGDD1 


                             
#define P_OUT   1
#define P_IN    0
#define PU_ON   1
#define PU_OFF  0

#define INIT_SPI_PINS_MACRO SCLK = 0; \
                            MOSI = 0; \
                            MISO = 1; \
                            SS = 1; \
                            SCLK_D = P_OUT; \
                            MOSI_D = P_OUT; \
                            MISO_D = P_IN; \
                            SS_D = P_OUT; \
                            MISO_PU = PU_ON;

#define LEFT_SPI_PINS_MACRO SCLK = 0; \
                            MOSI = 0; \
                            MISO = 0; \
                            SS = 0; \
                            SCLK_D = P_IN; \
                            MOSI_D = P_IN; \
                            MISO_D = P_IN; \
                            SS_D = P_IN; \
                            MISO_PU = PU_OFF; 
                            
#define SwSPI_Start() SCLK = 0; SS = 0;
#define SwSPI_Stop() SS = 1; SCLK = 1; MOSI = 0;                             
                            
#define MOSI_HOLD 5    //28.5MHz Bus Clock: 1  => SPI 285kHz
#define SCLK_HOLD 5    //                   5  => SPI 170kHz
                       //                  10  => SPI 115kHz
                                                      

// FUNCTION DECLARATION
void SwSPI_Wait(byte);
void SwSPI_Init(void);
void SwSPI_SendChar(byte tx_byte);
byte SwSPI_ReceiveChar(void);
byte SwSPI_ReadByte(byte RegAdd);
void SwSPI_WriteByte(byte RegAdd, byte val);

#endif //SW_SPI_H 
