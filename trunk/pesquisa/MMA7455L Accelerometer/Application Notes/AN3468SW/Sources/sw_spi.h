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

#ifndef SW_SPI_H
#define SW_SPI_H

#include "derivative.h"

// Pins definition
#define SCLK    PTCD_PTCD3
#define SCLK_D  PTCDD_PTCDD3
#define MOSI    PTCD_PTCD2
#define MOSI_D  PTCDD_PTCDD2
#define MISO    PTAD_PTAD1
#define MISO_D  PTADD_PTADD1
#define MISO_PU PTAPE_PTAPE1
#define SS      PTBD_PTBD3     
#define SS_D    PTBDD_PTBDD3  

                             
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
                            
#define MOSI_HOLD 5    //20MHz Bus Clock: 1  => SPI 200kHz
#define SCLK_HOLD 5    //                 5  => SPI 120kHz
                       //                10  => SPI  80kHz
                                                      

// FUNCTION DECLARATION
void SwSPI_Wait(byte);
void SwSPI_Init(void);
void SwSPI_SendChar(byte tx_byte);
byte SwSPI_ReceiveChar(void);
byte SwSPI_ReadByte(byte RegAdd);
void SwSPI_WriteByte(byte RegAdd, byte val);

#endif //SW_SPI_H 
