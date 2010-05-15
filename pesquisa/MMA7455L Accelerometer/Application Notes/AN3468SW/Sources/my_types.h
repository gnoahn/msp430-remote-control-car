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
* @file      my_types.h
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


#ifndef MY_TYPES_H
#define MY_TYPES_H

#include "derivative.h"

// ION registers
#define XOUTL 0x00     //00  10 bits output value X LSB (Read only)
#define XOUTH 0x01     //01  10 bits output value X MSB (Read only)
#define YOUTL 0x02     //02  10 bits output value Y LSB (Read only)
#define YOUTH 0x03     //03  10 bits output value Y MSB (Read only)
#define ZOUTL 0x04     //04  10 bits output value Z LSB (Read only)
#define ZOUTH 0x05     //05  10 bits output value Z MSB (Read only)
#define XOUT8 0x06     //06   8 bits Output value X (Read only)
#define YOUT8 0x07     //07   8 bits Output value Y (Read only)
#define ZOUT8 0x08     //08   8 bits Output value Z (Read only)
#define STATUS 0x09    //09  Status registers (Read only)
#define DETSRC 0x0A    //10  Detection source registers (Read only)
#define TOUT 0x0B      //11  Temperature output value (Optional)
#define RESERVED1 0x0C //12  Reserved
#define I2CAD 0x0D     //13  I2C device address (Bit[6:0]: Read only, Bit[7]: Read/Write)
#define USRINF 0x0E    //14  User information  (Optional, Read only)
#define WHOAMI 0x0F    //15  Who am I value (Optional, Read only)
#define XOFFL 0x10     //16  Offset drift X value (LSB) (Read/Write)
#define XOFFH 0x11     //17  Offset drift X value (MSB) (Read/Write)
#define YOFFL 0x12     //18  Offset drift Y value (LSB) (Read/Write)
#define YOFFH 0x13     //19  Offset drift Y value (MSB) (Read/Write)
#define ZOFFL 0x14     //20  Offset drift Z value (LSB) (Read/Write)
#define ZOFFH 0x15     //21  Offset drift Z value (MSB) (Read/Write)
#define MCTL  0x16     //22  Mode control (Read/Write)
#define INTRST 0x17    //23  Interrupt latch reset (Read/Write)
#define CTL1 0x18      //24  Control 1 (Read/Write)
#define CTL2 0x19      //25  Control 2 (Read/Write)
#define LDTH 0x1A      //26  Level detection threshold limit value (Read/Write)
#define PDTH 0x1B      //27  Pulse detection threshold limit value (Read/Write)
#define PW 0x1C        //28  Pulse duration value (Read/Write)
#define LT 0x1D        //29  Latency time value  (Read/Write)
#define TW 0x1E        //30  Time window for second pulse value(Read/Write)
#define RESERVED2 0x1F //31 Reserved




// Definition of new types   //Reference document: Data Sheet MMA7455L Rev1, 11/2007
typedef union {              //00 XOUTL  10 bits output value X LSB (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {                   // Read XOUTL latches XOUTH. 
    byte XOUTL_0 :1;         // Read XOUTH directly following XOUTL
    byte XOUTL_1 :1;
    byte XOUTL_2 :1;
    byte XOUTL_3 :1;
    byte XOUTL_4 :1;
    byte XOUTL_5 :1;
    byte XOUTL_6 :1;
    byte XOUTL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGXOUTL;

typedef union {              //01 XOUTH  10 bits output value X MSB (Read only)
  byte byte;
  struct {
    byte XOUTH_8 :1;
    byte XOUTH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGXOUTH;

typedef union {              //02 YOUTL  10 bits output value Y LSB (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {                   // Read YOUTL latches YOUTH
    byte YOUTL_0 :1;         // Read YOUTH directly following YOUTL
    byte YOUTL_1 :1;
    byte YOUTL_2 :1;
    byte YOUTL_3 :1;
    byte YOUTL_4 :1;
    byte YOUTL_5 :1;
    byte YOUTL_6 :1;
    byte YOUTL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGYOUTL;
                             //03 YOUTH  10 bits output value Y MSB (Read only)
typedef union {
  byte byte;
  struct {
    byte YOUTH_8 :1;
    byte YOUTH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGYOUTH;

typedef union {              //04 ZOUTL  10 bits output value Z LSB (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {                   // Read ZOUTL latches ZOUTH
    byte ZOUTL_0 :1;         // Read ZOUTH directly following ZOUTL
    byte ZOUTL_1 :1;
    byte ZOUTL_2 :1;
    byte ZOUTL_3 :1;
    byte ZOUTL_4 :1;
    byte ZOUTL_5 :1;
    byte ZOUTL_6 :1;
    byte ZOUTL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGZOUTL;

typedef union {              //05 ZOUTH  10 bits output value Z MSB (Read only)
  byte byte;
  struct {
    byte ZOUTH_8 :1;
    byte ZOUTH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGZOUTH;

typedef union {              //06 XOUT8   8 bits Output value X (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte XOUT8_0 :1;
    byte XOUT8_1 :1;
    byte XOUT8_2 :1;
    byte XOUT8_3 :1;
    byte XOUT8_4 :1;
    byte XOUT8_5 :1;
    byte XOUT8_6 :1;
    byte XOUT8_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGXOUT8;

typedef union {              //07 YOUT8   8 bits Output value Y (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte YOUT8_0 :1;
    byte YOUT8_1 :1;
    byte YOUT8_2 :1;
    byte YOUT8_3 :1;
    byte YOUT8_4 :1;
    byte YOUT8_5 :1;
    byte YOUT8_6 :1;
    byte YOUT8_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGYOUT8;

typedef union {              //08 ZOUT8   8 bits Output value Z (Read only)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte ZOUT8_0 :1;
    byte ZOUT8_1 :1;
    byte ZOUT8_2 :1;
    byte ZOUT8_3 :1;
    byte ZOUT8_4 :1;
    byte ZOUT8_5 :1;
    byte ZOUT8_6 :1;
    byte ZOUT8_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGZOUT8;
                             
typedef union {              //09 STATUS  Status registers (Read only)
  byte byte;
  struct {
    byte STATUS_DRDY :1;     //   DRDY: 1 data ready / 0 data not ready
    byte STATUS_DOVR :1;     //   DOVR: 1 data over written / 0 data not over written 
    byte STATUS_PERR :1;     //   PERR: 1 parity error in trim data then self test disabled / 0 parity error not detected in trim data
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGSTATUS;

typedef union {              //10 DETSR  Detection source registers (Read only)
  byte byte;
  struct {
    byte DETSR_LDX :1;       //   LDX:  1 Level detection event on X / 0 Level detection event not detected on X
    byte DETSR_LDY :1;       //   LDY:  1 Level detection event on Y / 0 Level detection event not detected on Y
    byte DETSR_LDZ :1;       //   LDZ:  1 Level detection event on Z / 0 Level detection event not detected on Z
    byte DETSR_PDX :1;       //   PDX:  1 Single Pulse detection event on X / 0 Single Pulse detection event not detected on X
    byte DETSR_PDY :1;       //   PDY:  1 Single Pulse detection event on Y / 0 Single Pulse detection event not detected on Y
    byte DETSR_PDZ :1;       //   PDZ:  1 Single Pulse detection event on Z / 0 Single Pulse detection event not detected on Z
    byte DETSR_INT2 :1;      //   INT2: Interrupt detected as assigned by detection control register (CTL1)
    byte DETSR_INT1 :1;      //   INT1: Interrupt detected as assigned by detection control register (CTL1)
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGDETSR;

typedef union {              //11 TOUT  Temperature output value (Optional)
  byte byte;
  struct {
    byte TOUT_0 :1;
    byte TOUT_1 :1;
    byte TOUT_2 :1;
    byte TOUT_3 :1;
    byte TOUT_4 :1;
    byte TOUT_5 :1;
    byte TOUT_6 :1;
    byte TOUT_7 :1;
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGTOUT;


typedef union {              //12  Reserved
  byte byte;
  struct {
    byte RESERVED1_0 :1;
    byte RESERVED1_1 :1;
    byte RESERVED1_2 :1;
    byte RESERVED1_3 :1;
    byte RESERVED1_4 :1;
    byte RESERVED1_5 :1;
    byte RESERVED1_6 :1;
    byte RESERVED1_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGRESERVED1; 

typedef union {              //13 I2CAD  I2C device address (Bit[6:0]: Read only, Bit[7]: Read/Write)
  byte byte;
  struct {
    byte DAD_0 :1;           //   DAD[6:0]: I2C device address 
    byte DAD_1 :1;
    byte DAD_2 :1;
    byte DAD_3 :1;
    byte DAD_4 :1;
    byte DAD_5 :1;
    byte DAD_6 :1;
    byte I2CDIS_7 :1;        //  I2CDIS: 1 I2C disabled / 0 I2C and SPI available  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGI2CAD;

typedef union {              //14 USRINF  User information  (Optional, Read only)
  byte byte;
  struct {
    byte USRINF_0 :1;
    byte USRINF_1 :1;
    byte USRINF_2 :1;
    byte USRINF_3 :1;
    byte USRINF_4 :1;
    byte USRINF_5 :1;
    byte USRINF_6 :1;
    byte USRINF_7 :1;
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGUSRINF;

typedef union {              //15 WHOAMI  Who am I value (Optional, Read only)
  byte byte;
  struct {
    byte WHOAMI_0 :1;
    byte WHOAMI_1 :1;
    byte WHOAMI_2 :1;
    byte WHOAMI_3 :1;
    byte WHOAMI_4 :1;
    byte WHOAMI_5 :1;
    byte WHOAMI_6 :1;
    byte WHOAMI_7 :1;
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGWHOAMI;

typedef union {              //16 XOFFL  Offset drift X value (LSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte XOFFL_0 :1;
    byte XOFFL_1 :1;
    byte XOFFL_2 :1;
    byte XOFFL_3 :1;
    byte XOFFL_4 :1;
    byte XOFFL_5 :1;
    byte XOFFL_6 :1;
    byte XOFFL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGXOFFL;

typedef union {              //17 XOFFH  Offset drift X value (MSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte XOFFH_8 :1;
    byte XOFFH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGXOFFH;

typedef union {              //18 YOFFL  Offset drift Y value (LSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte YOFFL_0 :1;
    byte YOFFL_1 :1;
    byte YOFFL_2 :1;
    byte YOFFL_3 :1;
    byte YOFFL_4 :1;
    byte YOFFL_5 :1;
    byte YOFFL_6 :1;
    byte YOFFL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGYOFFL;

typedef union {              //19 YOFFH  Offset drift Y value (MSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte YOFFH_8 :1;
    byte YOFFH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGYOFFH;

typedef union {              //20 ZOFFL  Offset drift Z value (LSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte ZOFFL_0 :1;
    byte ZOFFL_1 :1;
    byte ZOFFL_2 :1;
    byte ZOFFL_3 :1;
    byte ZOFFL_4 :1;
    byte ZOFFL_5 :1;
    byte ZOFFL_6 :1;
    byte ZOFFL_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGZOFFL;

typedef union {              //21 ZOFFH  Offset drift Z value (MSB) (Read/Write)
  byte byte;                 // signed byte data (2's compliment)
  struct {
    byte ZOFFH_8 :1;
    byte ZOFFH_9 :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;
    byte  :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGZOFFH;

typedef union {              //22 MCTL  Mode control (Read/Write)
  byte byte;
  struct {
    byte MOD0 :1;            //   MOD[1:0]  00 StandBy / 01 Measure / 10 Level /11 Pulse
    byte MOD1 :1;            //   
    byte GLVL0 :1;           //   GLVL[1:0] 00 8g range (detection is 8g range)/ 01 2g range (detection is 8g range)/ 10 4g range (detection is 8g range)
    byte GLVL1 :1;           //   
    byte ST0N :1;            //   STON      1 Self Test enabled / 0 Self Test disabled
    byte SPI3W :1;           //   SPI3W     1 SPI 3 wire mode / 0 SPI 4 wire mode
    byte DRPD :1;            //   DRPD      1 Data Ready is output to INT1/DRDY pin / 0 Data Ready is not output to INT1/DRDY pin   
    byte :1;               
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGMCTL;

typedef union {              //23 INTRST  Interrupt latch reset (Read/Write)
  byte byte;
  struct {
    byte CLRINT1 :1;         //   CLRINT1   1: Clear INT1 and LDX/LDY/LDZ bits or PDX/PDY/PDZ bits in detection source register (DETSR) 
    byte CLRINT2 :1;         //                depending on detection control register (CTL1) 
    byte :1;                 //             0: Do not clear INT1 and LDX/LDY/LDZ bits or PDX/PDY/PDZ bits in detection source register (DETSR)
    byte :1;                 //   CLRINT1   1: Clear INT1 and LDX/LDY/LDZ bits or PDX/PDY/PDZ bits in detection source register (DETSR) 
    byte :1;                 //                depending on detection control register (CTL1)
    byte :1;                 //             0: Do not clear INT1 and LDX/LDY/LDZ bits or PDX/PDY/PDZ bits in detection source register (DETSR)
    byte :1;                 
    byte :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGINTRST;
                            
typedef union {
  byte byte;                 //24 CTL1  Control 1 (Read/Write)
  struct {
    byte INTPIN :1;          //   INTPIN    1: INT2 pin routed to INT1 register and INT1 pin routed to INT2 register / 0: INT1 pin routed to INT1 register and INT1 pin routed to INT1 register 
    byte INTREG0 :1;         //   INTREG[1:0]   00 INT1: Level / INT2: Pulse
    byte INTREG1 :1;         //                 01 INT1: Pulse / INT2: Level
    byte XDA :1;             //                 10 INT1: Single Pulse / Pulse
    byte YDA :1;             //                    Note: If second pulse is not detected, no automatic reset. It is required to reset through CLR_INT2
    byte ZDA :1;             //   XDA,YDA,ZDA   1: axis disabled for detection / 0: axis enabled for detection 
    byte THOPT :1;           //   THOPT         1: Threshold value for level detection is positive/negative / 0: Threshold value for level detection is absolute
    byte DFBW :1;            //   DFBW          1: Digital filter band is 125Hz (DRDY: 250Hz) / 0: Digital filter band is 62.5Hz (DRDY: 125Hz)  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGCTL1; 

typedef union {              //25 CTL2  Control 2 (Read/Write)
  byte byte;
  struct {
    byte LDPL :1;            //   LDPL          1: Level detection is negative and "AND" 3 axes / 0: Level detection is "OR" 3 axes
    byte PDPL :1;            //   PDPL          1: Pulse detection is negative and "AND" 3 axes / 0: Pulse detection is "OR" 3 axes
    byte DRV0 :1;            //   DRV0          1: Strong drive strength on SDA/SDO pin0 / 0: Standard drive strength on SDA/SDO pin
    byte :1;
    byte :1;
    byte :1;
    byte :1;
    byte :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGCTL2; 

typedef union {              //26 LDTH  Level detection threshold limit value (Read/Write)
  byte byte;                 // THOPT=0 => 7bits unsigned and LDTH[7]=0
  struct {                   // THOPT=1 => 8bits signed and LDTH[7]=0
    byte LDTH_0 :1;
    byte LDTH_1 :1;
    byte LDTH_2 :1;
    byte LDTH_3 :1;
    byte LDTH_4 :1;
    byte LDTH_5 :1;
    byte LDTH_6 :1;
    byte LDTH_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGLDTH; 

typedef union {              //27 PDTH  Pulse detection threshold limit value (Read/Write)
  byte byte;                 // 7bits unsigned, XPDTH should be 0 
  struct {
    byte PDTH_0 :1;         
    byte PDTH_1 :1;
    byte PDTH_2 :1;
    byte PDTH_3 :1;
    byte PDTH_4 :1;
    byte PDTH_5 :1;
    byte PDTH_6 :1;
    byte XPDTH_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGPDTH; 

typedef union {              //28 PW  Pulse duration value (Read/Write)
  byte byte;                 // 1LSB=0.5ms
  struct {
    byte PW_0 :1;
    byte PW_1 :1;
    byte PW_2 :1;
    byte PW_3 :1;
    byte PW_4 :1;
    byte PW_5 :1;
    byte PW_6 :1;
    byte PW_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGPW; 

typedef union {              //29 LT  Latency time value  (Read/Write)
  byte byte;                 // 1LSB=1ms
  struct {
    byte LT_0 :1;
    byte LT_1 :1;
    byte LT_2 :1;
    byte LT_3 :1;
    byte LT_4 :1;
    byte LT_5 :1;
    byte LT_6 :1;
    byte LT_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGLT; 

typedef union {              //30 TW  Time window for second pulse value(Read/Write)
  byte byte;                 // 1LSB=1ms
  struct {
    byte TW_0 :1;
    byte TW_1 :1;
    byte TW_2 :1;
    byte TW_3 :1;
    byte TW_4 :1;
    byte TW_5 :1;
    byte TW_6 :1;
    byte TW_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGTW; 

typedef union {              //31 Reserved
  byte byte;
  struct {
    byte RESERVED2_0 :1;
    byte RESERVED2_1 :1;
    byte RESERVED2_2 :1;
    byte RESERVED2_3 :1;
    byte RESERVED2_4 :1;
    byte RESERVED2_5 :1;
    byte RESERVED2_6 :1;
    byte RESERVED2_7 :1;  
  } Bits;
  struct {
    byte Reg :8;
  } MergedBits;
} READ_REGRESERVED2; 

#endif //MY_TYPES_H





