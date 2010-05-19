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
* @file      main.c
*
* @author    R20246
* 
* @version   1.0.0.0
* 
* @date      MAR-6-2008
* 
* @brief     ION (MMA7455L)
*
*******************************************************************************
*
* 
*
******************************************************************************/

    #include <MC9S08QE128.h>
    
    void mcu_init(void)
    { 
   
   /* SOPT1 - System Options Register 1            
      COPE: COP Watchdog Enable                  0 
      COPT: COP Watchdog Timeout                 1 
      STOPE: Stop Mode Enable                    0 
                                                 0 
                                                 0 
      RSTOPE: RSTO Pin Enable                    0 
      BKGDPE: Background Debug Mode Pin Enable   1 
      RSTPE:  RESET Pin Enable                   0 */
   SOPT1 = 0x42;
   
   /*
   ICSOUT: This clock source is used as the CPU clock and is divided by 2 to generate the peripheral bus clock. 
           Control bits in the ICS control registers determine which of three clock sources is connected:
           - Internal reference clock
           - External reference clock
           - Frequency-locked loop (FLL) output
   ICSLCLK: This clock source is derived from the digitally controlled oscillator (DCO) of the ICS when 
            the ICS is configured to run off of the internal or external reference clock. */
            
   /* SCGC1 - System Clock Gating Control 1 Register
      TPM3      TPM3 Clock Gate Control          0
      TPM2      TPM2 Clock Gate Control          0
      TPM1      TPM1 Clock Gate Control          0
      ADC       ADC Clock Gate Control           1
      IIC2      IIC2 Clock Gate Control          0
      IIC1      IIC1 Clock Gate Control          0
      SCI2:     SCI2 Clock Gate Control          0
      SCI1:     SCI1 Clock Gate Control          1 */  
   SCGC1=0b01110001;
   
   /* SCGC2 - System Clock Gating Control 2 Register
      SPI1:     SPI1 Clock Gate Control 
      SPI2:     SPI2 Clock Gate Control 
      RTC:      RTC Clock Gate Control 
      ACMP:     ACMP Clock Gate Control 
      KBI:      KBI Clock Gate Control 
      IRQ:      IRQ Clock Gate Control 
      FLS:      FLASH Register Clock Gate Control
      DBG:      DBG Clock Gate Control */
   SCGC2=0b00000000;
   
   /* ICSC1 - ICS Control Register 1                
      CLKS1: Clock Source Select, bit 1          0  
      CLKS0: Clock Source Select, bit 0          1
              00 Output of FLL is selected.
              01 Internal reference clock is selected.
              10 External reference clock is selected.
              11 Reserved, defaults to 00.  
      => Internal Reference Clock selected           
       FEI FLL ENGAGED INTERNAL                    
     RDIV2: Reference Divider, bit 2             0  
     RDIV1: Reference Divider, bit 1             0  
     RDIV0: Reference Divider, bit 0             0  
     => Reference Divide Factor = 1                
     IREFS: Internal Reference Select            1  
     IRCLKEN: Internal Reference Clock Enable    0  
     IREFSTEN: Internal Reference Stop Enable    0 */                 
   ICSC1=0b00000100;     //CASE1: FEI 29MHz (output of FLL is selected and Internal Reference Selected) 
   //ICSC1 = 0b00000000; //CASE2: FEE  2MHz (output of FLL is selected)
   //ICSC1 = 0b10111000; //CASE3: FBE low frequency (external reference clock selected and external reference divided by 5)
   //ICSC1 = 0b01000000; //CASE4: FBI low frequency (internal reference clock is selected) */
   
   /* ICS2: Control Register 2                      
      BDIV1:    Bus Frequency Divider, bit 1     0  
      BDIV0:    Bus Frequency Divider, bit 0     0  
      RANGE:    Frequency Range Select           0  
      HGO:      High Gain Oscillator Select      0    
      LP:       Low Power Select                 0   
      EREFS:    External Reference Select        0    
      ERCLKEN:  External Reference Enable        0   
      EREFSTEN: External Reference Stop Enable   0 */  
   ICSC2=0b00000000;     //CASE1: FEI (bus frequency divided by 1)
   //ICSC2 = 0b10000111; //CASE2: FEE (divides selected clock by 4 and external reference is selected)  
   //ICSC2 = 0b00000000; //CASE3: FBE (bus frequency divided by 1)
   //ICSC2 = 0b00000000; //CASE4: FBI (divides selected clock by 1) */
   
   ICSTRM = *(unsigned char*far)0xFFAF; /* Initialize ICSTRM register from a non volatile memory */
   
   /* ICSSC - ICS Status and Control Register 
      DRST_DRS1 DCO Range Status/Range Select, bit 1                 1
      DRST_DRS0 DCO Range Status/Range Select, bit 0                 0
      DMX32     DCO Maximum frequency with 32.768 kHz reference      1
      IREFST    Internal Reference Status                            0
      CLKST1    Clock Mode Status, bit 1                             0
      CLKST0    Clock Mode Status, bit 0                             0
      OSCINIT   OSC Initialization                                   0
      FTRIM:    ICS Fine Trim                                        0 
      => Digitally controlled oscillator (DCO): DCO=59.77MHz                                     */
   ICSSC = (*(unsigned char*far)0xFFAE) | 0xA0;  //CASE1: FEI /* Initialize ICSSC register from a non volatile memory */   
   //ICSSC = (*(unsigned char*far)0xFFAE) | 0x00; //CASE2: FEE, CASE3: FBE and CASE4: FBI
     
   /* SPMSC1 System Power Management Status and Control 1 Register 
      LVDF      Low-Voltage Detect Flag 
      LVDACK    Low-Voltage Detect Acknowledge 
      LVDIE     Low-Voltage Detect Interrupt Enable
      LVDRE     Low-Voltage Detect Reset Enable
      LVDSE     Low-Voltage Detect Stop Enable 
      LVDE:     Low-Voltage Detect Enable
      BGBE:     Bandgap Buffer Enable                                                            */
   //SPMSC1 = SPMSC1_LVDRE_MASK | SPMSC1_LVDE_MASK ;
      //SPMSC1=0x00;  //Bandgap disabled
      SPMSC1=0x01;    //Bandgap enabled
      
    /* SPMSC2 System Power Management Status and Control 2 Register
      LPR:      Low Power Regulator Control 
      LPRS:     Low Power Regulator Status 
      LPWUI:    Low Power Wake Up on Interrupt
      
      PPDF:     Partial Power Down Flag
      PPDACK:   Partial Power Down Acknowledge 
      PPDE:     Partial Power-Down Enable
      PPDC:     Partial Power Down Control 
      */
      SPMSC2=0x00;

    /* SRS System Reset Status Register (SRS)   
       POR:    Power-On Reset                                1
       PIN:    External Reset Pin                            0
       COP:    Computer Operating Properly (COP) Watchdog    0
       ILOP:   Illegal Opcode                                1
        -
        -                                                    
       LVD:     Low Voltage Detect                           1
        -                                                                                       */                                              
      SRS=0b10010010; /* Disable wake up timer */
      
    }