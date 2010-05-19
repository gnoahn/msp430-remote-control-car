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
* @file      adc.c
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

#include "derivative.h"
#include "adc.h"

unsigned int VDD,VDD2,T;
unsigned long Temp_Voltage25_12Bits,Temp_ColdSlope12Bits,Temp_HotSlope12Bits; 
extern byte  temp_QE128H,temp_QE128L,VDD_QE128H,VDD_QE128L;
extern byte x_hermesH,x_hermesL,y_hermesH,y_hermesL,z_hermesH,z_hermesL;
extern unsigned long temp_QE128;
unsigned long measuredtemp;             

void initADC_Bits(byte ADC_Bits)
{
  /* ADCCFG - Configuration Register 
     ADLPC:      Low Power Configuration            0          
                 0 High speed configuration
                 1 Low power configuration
     ADIV1:      Clock Divide Select Bit 1          
     ADIV0:      Clock Divide Select Bit 0          
                     Divide Ratio
                 00       1
                 01       2
                 10       4                        10
                 11       8
     ADLSMP:     Long Sample Time Configuration     1
                 0 Short sample time
                 1 Long sample time
     MODE1:      Conversion Mode Selection Bit 1
     MODE0:      Conversion Mode Selection Bit 0 
                 00   8Bits
                 01  12Bits                        00
                 10  10Bits
     ADICLK1:    Input Clock Select Bit 1 
     ADICLK0:    Input Clock Select Bit 0               
                 00 Bus clock                      00
                 01 Bus clock divided by 2
                 10 Alternate clock (ALTCLK)
                 11 Asynchronous clock (ADACK)
     */
  
  ADCCFG=(0b01010000|ADC_Bits); /* 0 High speed, 10 divide 4,  1 Long sample size, 01 12Bits, 00 ADCLK=BUS CLOCK */
  ADCSC2=0b00000000;   /* Software trigger selected */
  APCTL1=0b00000000; /* All GPIO, ADC disabled */  //AD1
  APCTL2=0b00000000; /* All GPIO, ADC disabled */  //AD8 & AD9
  APCTL3=0b00000000; /* All GPIO, ADC disabled */  //AD16)               
}

void initADC(void) 
{ 
  ADCCFG=0b01010100; /* 0 High speed, 10 divide 4,  1 Long sample size, 01 12Bits, 00 ADCLK=BUS CLOCK/4 */
  ADCSC2=0b00000000;   /* Software trigger selected */
  APCTL1=0b00000000; /* All GPIO, ADC disabled */  //AD1
  APCTL2=0b00000000; /* All GPIO, ADC disabled */  //AD8 & AD9
  APCTL3=0b00000000; /* All GPIO, ADC disabled */  //AD16) 
}
   
void ReadADC_Hermes(void) 
{
  ADCSC1=0b00100001;   /* Continuous Conversion, AD1 PTA1 (16) */     
  while(ADCSC1_COCO==0);
  x_hermesH=ADCRH;      //Read H first then L
  x_hermesL=ADCRL;     
    
  ADCSC1=0b00101000;   /* Continuous Conversion, AD8 PTA6 (22) */   
  while(ADCSC1_COCO==0);
  y_hermesH=ADCRH;      //Read H first then L
  y_hermesL=ADCRL;  
 
  ADCSC1=0b00101001;   /* Continuous Conversion, AD9 PTA7 (24) */     
  while(ADCSC1_COCO==0);
  z_hermesH=ADCRH;      //Read H first then L
  z_hermesL=ADCRL;    
}

void initTemp_QE128(void) //ADC 12Bits, calibration to VDD instead of reference VDD 3V  
{
byte i;               //Up to 250 if initTemp_QE128() is before for(;;) of main.c 
byte count=250;       //10  if initTemp_QE128() is in for(;;) of main.c 
unsigned long tmp;

tmp=0;

//VDD_QE128();
for(i=0;i<count;i++) 
{
tmp+=VDD_Temp_QE128();
}

VDD2=(unsigned int)(tmp/count);

Temp_ColdSlope12Bits=6740370/VDD2;   // VDD here is VDD*1000
                                   // -40°C to 25°C 1.646mV/°C 
                                   // x1000 1.646*4095*1000=6740370
Temp_HotSlope12Bits=7244055/VDD2;    // VDD here is VDD*1000
                                   //  25°C to 85°C 1.769mV/°C 
                                   // x1000 1.769*4095*1000=7244055
                                           
Temp_Voltage25_12Bits=2871414/VDD2; // VDD here is VDD*1000         
                                   //  25°C 701.2mV 
                                   //  0.7012*4095*1000=2871414 
}

void Temp_QE128(void)     //ADC 12Bits, measurement of temperature using QE128
{
  signed long tmp;
  
  initADC_Bits(ADC_12Bits);   
    
  ADCSC1=0b00111010;   /* Continuous Conversion, AD26 Temperature Sensor */     
  while(ADCSC1_COCO==0);
  tmp=ADCR; 
  measuredtemp=tmp;  

  if (tmp>Temp_Voltage25_12Bits) //Cold slope to be used
      {
        tmp=((tmp-Temp_Voltage25_12Bits)*1000);
        tmp=(tmp/Temp_ColdSlope12Bits);
        tmp=25-tmp;
        tmp+=100;
        
      }
      else                        //Hot slope to be used
      {
        tmp=((Temp_Voltage25_12Bits-tmp)*1000);  
        tmp=(tmp/Temp_HotSlope12Bits);
        tmp+=25;                                
        tmp+=100;
      }
      
  temp_QE128L=(unsigned char)(tmp&0x00FF);
  temp_QE128H=(unsigned char)(tmp>>8);  

  initADC_Bits(ADC_Def); 
}

void VDD_QE128(void)   //ADC 12Bits, measurement of (VDD*1000)
{
  unsigned int tmp;
  
  initADC_Bits(ADC_12Bits);   
  
 /* ADCSC1 - Status and Control Register 1
    COCO:       Conversion Complete Flag 
    AIEN:       Interrupt Enable 
    ADCO:       Continuous Conversion Enable - ADCO is used to enable continuous conversions 
    ADCH4:      Input Channel Select Bit 4 
    ADCH3:      Input Channel Select Bit 3 
    ADCH2:      Input Channel Select Bit 2 
    ADCH1:      Input Channel Select Bit 1 
    ADCH0:      Input Channel Select Bit 0  */
    
  ADCSC1=0b00111011;   /* Continuous Conversion, AD27 Internal Bandgap */     
  while(ADCSC1_COCO==0);
  
  tmp=(unsigned int)(4770675/ADCR);    //Average applicable to all DEMOQE
  //tmp=(unsigned int)(4774823/ADCR);  //Applicable specifically for PE5021231 
  //tmp=(unsigned int)(4764602/ADCR);  //Applicable specifically for PE5021178 
  //tmp=(unsigned int)(4772800/ADCR);  //Applicable specifically for PE5021116 
    
  VDD=tmp;
  VDD_QE128L=tmp&0x00FF;
  VDD_QE128H=tmp>>8;
  
  initADC_Bits(ADC_Def); 
}

  // DEMOQE SN25 PE5021116 (28 544 700Hz) VDD=3.140V (at MCU)  ADC_12Bits Bandgap BG=1520 VDD/4095=BG/1520 BG=3.140*1520/4095=1.165519    
  //  ADC  12Bits
  //        VDD*10=1.165519*1000*4095/ADCR=4772800/ADCR   
  // DEMOQE SN21 PE5021178 (28 544 700Hz) VDD=3.106V (at MCU)  ADC_12Bits Bandgap BG=1534 VDD/4095=BG/1534 BG=3.106*1534/4095=1.163517     
  //  ADC  12Bits
  //        VDD*10=1.163517*1000*4095/ADCR=4764602/ADCR    
  // DEMOQE SN9 PE5021231 (28 544 700Hz) VDD=3.196V (at MCU)  ADC_12Bits Bandgap BG=1494 VDD/4095=BG/1494 BG=3.196*1494/4095=1.166013 
  //  ADC  12Bits
  //        VDD*1000=1.166013*1000*4095/ADCR=4774823/ADCR 
  // Average used on all DEMOQE:
  // ===========================  
  //  ADC   8Bits
  //        VDD*1000=1.165*1000*255/ADCR=297075/ADCR       
  //  ADC  10Bits
  //        VDD*1000=1.165*1000*1023/ADCR=1191795/ADCR      
  //  ADC  12Bits
  //        VDD*1000=1.165*1000*4095/ADCR=4770675/ADCR  

  unsigned int VDD_Temp_QE128(void)   //ADC 12Bits, measurement of (VDD*1000)
{
  unsigned int tmp;
  
  initADC_Bits(ADC_12Bits);   
  
 /* ADCSC1 - Status and Control Register 1
    COCO:       Conversion Complete Flag 
    AIEN:       Interrupt Enable 
    ADCO:       Continuous Conversion Enable - ADCO is used to enable continuous conversions 
    ADCH4:      Input Channel Select Bit 4 
    ADCH3:      Input Channel Select Bit 3 
    ADCH2:      Input Channel Select Bit 2 
    ADCH1:      Input Channel Select Bit 1 
    ADCH0:      Input Channel Select Bit 0  */
    
  ADCSC1=0b00111011;   /* Continuous Conversion, AD27 Internal Bandgap */     
  while(ADCSC1_COCO==0);
  
  tmp=(unsigned int)(4770675/ADCR);    //Average applicable to all DEMOQE
  //tmp=(unsigned int)(4774823/ADCR);  //Applicable specifically for PE5021231 
  //tmp=(unsigned int)(4764602/ADCR);  //Applicable specifically for PE5021178 
  //tmp=(unsigned int)(4772800/ADCR);  //Applicable specifically for PE5021116 
    
  VDD=tmp;
  VDD_QE128L=tmp&0x00FF;
  VDD_QE128H=tmp>>8;
  
  initADC_Bits(ADC_Def);
  
  return(VDD); 
}


