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
* @file      mcu_init.c
*
* @author    R20246
* 
* @version   2.0.0.1
* 
* @date      FEB-14-2008
* 
* @brief     ION (MMA7455L) - SPI Communication Protocol
*
*******************************************************************************
*
* 
*
******************************************************************************/

#include <MC9S08GB60.h>               



void MCU_Init(void)
{
    
   /* Configure FEI FLL ENGAGED INTERNAL / Bus Frequency: 20MHz (19 995 429Hz)*/
    
   /* ICGC1: Reserved,RANGE=0,REFS=1,CLKS1=0,CLKS0=1,OSCSTEN=0,Reserved,Reserved*/
	 /* RANGE Frequency Range Select,RANGE=0: Oscillator configured for low frequency(P=64)*/
	 /* REFS External Reference Select, REFS=1: crystal or resonator requested*/
	 /* CLKS Clock Mode Select
	    00 Self-clocked (SCM, SELF CLOCK MODE, fICGout=fICGDCLK/R)
	    01 FLL engaged,internal reference  (FEI, FLL ENGAGED INTERNAL, fICGout=(fIRG/7)*64*N/R)
	    10 FLL bypassed,external reference (FBE, FLL BYPASSED EXTERNAL,fICGout=fext/R)
	    11 FLL engaged,external reference  (FEE, FLL ENGAGED EXTERNAL, fICGout=fext*P*N/R)
	    CLKS=01: FLL engaged,internal reference, FEI FLL ENGAGED INTERNAL*/
	 /* OSCSTEN Enable Oscillator in Off Mode, OSCSTEN=0: Oscillator disabled when ICG is off*/
      ICGC1 = 0x28;
    
   /* ICGC2: LOLRE=0,MFD2=1,MFD1=1,MFD0=1,LOCRE=0,RFD2=0,RFD1=0,RFD0=0 */  
	 /* LOLRE: Loss of Lock Reset Enable,LOLRE=0: Generate an interrupt request on loss of lock*/
	 /* MFD2,MFD1,MFD0: Multiplication factor, MFD=111 - Multiplication factor (N)= 18*/
	 /* LOCRE: Loss of Clock Reset Enable,LOCRE=0:Generate an interrupt request on loss of clock*/
	 /* RFD2,RFD1,RFD0: Reduced Frequency Divider, 000 R=1*/
   /* FEI, FLL ENGAGED INTERNAL, fICGout=(fIRG/7)*64*N/R) */ 
   /* fICGout=(fIRG/7)*64*N/R */
   /* fIRG=243kHz */
   /* Range=0 => P=64 */ 
   /* MFD=111 => N=18 */
   /* RFD=0 => R=1 */
   /* fICGout=243kHz/7*64*18/1= 40MHz (39 990 860Hz)*/
   /* Bus Frequency = 40MHz/2 = 20MHz (19 995 429Hz)*/
      ICGC2 = 0x70;
    
   /* FEI, FLL ENGAGED INTERNAL, fICGout=(fIRG/7)*64*N/R) */ 
   /* fICGout=(fIRG/7)*64*N/R */
   /* fIRG=243kHz */
   /* Range=0 => P=64 */ 
   /* MFD=101 => N=14 */
   /* RFD=001 => R=2 */
   /* fICGout=243kHz/7*64*14/2= 15.5MHz (15 552 000Hz)*/
   /* Bus Frequency = 15.5MHz/2 = 7.8MHz (7 776 000Hz)*/
   // ICGC2 = 0x51;   
    while(!ICGS1_LOCK);  
    SOPT = 0x73; 
    
    SRTISC=SRTISC&~0x07; /* Disable wake up timer */
    SPMSC2=0x00; 
    SPMSC1 = SPMSC1_LVDE_MASK | SPMSC1_LVDRE_MASK ;

}


