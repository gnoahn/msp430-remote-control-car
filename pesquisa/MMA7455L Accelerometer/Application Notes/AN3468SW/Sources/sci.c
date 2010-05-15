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
* @file      sci.c
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

#include "sci.h"

/**************************************************************
*	Function: 	initSCI 
*	Parameters: None 
*	Return:		 
*
*	
**************************************************************/
void SCI_Init(void)
{
                        /* Baud Rate selection */
                        /* SCI Baud Rate = Bus Clock / (16 x BR)*/
                        /* Baud Rate mismatch: +/-4.5% 8-Bit data format */
                        /*                     +/-4%   9-Bit data format */
                        
    SCI1BDH=0x00;       /*Baud Rate selection*/
                        
    SCI1BDL=0x0B;       /* CASE: 115200 Bauds */
                        /* SCIBDL=0B 115200 Bauds */                        
    					          /* SCI Baud Rate = 20MHz / (16 x 11) = 113636  */ 
    					          /* Debugger: 20 000 000Hz */                
   					  
    SCI1C1=0x00;        	  						
 	  SCI1C2_TE=0x01;	  	/*Enable SCI Transmission */
    SCI1C3 = 0x00;
    
    SCI1S2 = 0x00;  
}


/**************************************************************
*	Function: 	Tx_byte_SCI 
*	Parameters: byte 
*	Return:		 
*
*	
**************************************************************/
void SCI_TxByte(unsigned char byte)
{	
  unsigned char Temp;
  
  while (!SCI1S1_TC);
	Temp = SCI1S1;
	SCI1D=byte;		
}

