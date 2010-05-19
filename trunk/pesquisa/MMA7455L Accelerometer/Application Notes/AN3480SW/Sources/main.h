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
* @file      main.h
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

#define OUTPUT 1
#define INPUT 0

#define PULL_ON 1
#define PULL_OFF 0

// ION POWER MANAGEMENT
#define ION_AVDD_MASK 0xF0
#define ION_DVDD_MASK 0x04 
#define ION_AVDD_INIT       PTDD &= ~ION_AVDD_MASK; PTDDD |= ION_AVDD_MASK;  
#define ION_AVDD_ON PTDD |= ION_AVDD_MASK;
#define ION_AVDD_OFF PTDD &= ~ION_AVDD_MASK;

#define ION_DVDD_INIT PTBD &= ~ION_DVDD_MASK; PTBDD |= ION_DVDD_MASK;                            
#define ION_DVDD_ON  PTBD |= ION_DVDD_MASK;
#define ION_DVDD_OFF PTBD &= ~ION_DVDD_MASK;                            


/* INTERCONNECT TO EVALUATION BOARD   //Reference document: Data Sheet MMA7455L Rev1, 11/2007

         ION                                M68DEMO908GB60E    M68EVB908GB60E                                             
    
    1    DVDD                               PTB2               PTB2
              Digital Power for I/O (input) 

    2    GND                                GND                GND
              Ground (input)

    3    N/C                                N/C (GND)          N/C (GND) 
              No connection or ground (input)

    4    IADDR0                             GND                GND
              I2C Address Bit 0 (input)

    5    GND                                GND                GND
              Ground (optional, input)

    6    AVDD                               PTD4 to 7          PTD4 to 7
              Analog Power (input)

    7    CS                                 PTB3               PTB3
              SPI Enable (0, input)        
              I2C Enable (1, input)

    8    INT1/DRDY                          PTA0                 PTA0
              Interrupt1/Data Ready (output)

    9    INT2                               PTA2               PTA2 (KBI possible) 
              Interrupt2 (output) 

    10   N/C                                N/C (GND)          N/C (GND) 
              No connection or ground (input)

    11   Reserved                           GND                GND
              Connect to ground (input)

    12   SDO                                PTA1               PTA1
              SPI Serial Data Output (SDO, output) 
     
    13   SDA/SDI/SDO                        PTC2               PTC2
              I2C Serial Data (SDA, open drain)
              SPI Serial Data Input (SDI, input)
              3 Wire Serial Data Output (SDO, output)

    14   SCL/SPC                            PTC3               PTC3                                                                                                                                                        
              I2C Serial Clock (SCL, input)
              SPI Serial Clock (SPC, input)                                                                           */



// BUTTON DECLARATION
#define BUTTON0 PTAD_PTAD4
#define BUTTON1 PTAD_PTAD5
#define BUTTON2 PTAD_PTAD6
#define BUTTON3 PTAD_PTAD7
#define INIT_BUTTON0  PTADD_PTADD4 = 0; PTAPE_PTAPE4 = 1; 
#define INIT_BUTTON1  PTADD_PTADD5 = 0; PTAPE_PTAPE5 = 1;
#define INIT_BUTTON2  PTADD_PTADD6 = 0; PTAPE_PTAPE6 = 1;
#define INIT_BUTTON3  PTADD_PTADD7 = 0; PTAPE_PTAPE7 = 1;
#define INIT_BUTTONS_MACRO INIT_BUTTON0; \
                           INIT_BUTTON1; \
                           INIT_BUTTON2; \
                           INIT_BUTTON3; 
                       
// ION INTERRUPT DECLARATION
#define INT1 PTAD_PTAD0      
#define INT1_D PTADD_PTADD0  
#define INT1_P PTAPE_PTAPE0
                                                          
#define INT2 PTAD_PTAD2      
#define INT2_D PTADD_PTADD2  
#define INT2_P PTAPE_PTAPE2

#define INIT_INT_MACRO INT1 = 0; \
                       INT1_D = INPUT; \
                       INT1_P = PULL_ON; \
                       INT2 = 0; \
                       INT2_D = INPUT; \
                       INT2_P = PULL_ON;
                       
//LED DEFINITION 	 
#define LED1 PTFD_PTFD0
#define LED2 PTFD_PTFD1
#define LED3 PTFD_PTFD2
#define LED4 PTFD_PTFD3
#define LED1_D PTFDD_PTFDD0
#define LED2_D PTFDD_PTFDD1
#define LED3_D PTFDD_PTFDD2
#define LED4_D PTFDD_PTFDD3
#define INIT_LED_MACRO LED1 = 1; \
                       LED1_D = 1; \
                       LED2 = 1; \
                       LED2_D = 1; \
                       LED3 = 1; \
                       LED3_D = 1; \
                       LED4 = 1; \
                       LED4_D = 1;
              

#define LED1_ON LED1=0;                   /* Set PTF0 Low (LED1 ON) */
#define LED2_ON LED2=0;                   /* Set PTF1 Low (LED2 ON) */
#define LED3_ON LED3=0;                   /* Set PTF2 Low (LED3 ON) */
#define LED4_ON LED4=0;                   /* Set PTF3 Low (LED4 ON) */

#define LED1_OFF LED1=1;                  /* Set PTF0 High (LED1 ON) */
#define LED2_OFF LED2=1;                  /* Set PTF1 High (LED2 ON) */
#define LED3_OFF LED3=1;                  /* Set PTF2 High (LED3 ON) */
#define LED4_OFF LED4=1;                  /* Set PTF3 High (LED4 ON) */
           

// ION MODES (MEASURE)
#define ION_MEASURE 1                     // Mode measure  

//ION G_SELECT VALUES
#define ION_G2 0b01                       // 2g range        
#define ION_G4 0b10                       // 4g range
#define ION_G8 0b00                       // 8g range

//ION AUTOCALIBRATION PROCESS
#define MAX_CNT_RUNS 55  


// FUNCTION DECLARATION
void main(void);
void CheckButt0(void);
void CheckButt1(void);
void CheckButt2(void);
void CheckButt3(void);
void Delay_Xms(byte);
void Read_AccelerometerIon(void);
void Read_AccelerometerIon_Array(void);
void ReadAllIonRegisters(void);
void Init_SensorIon(void);
void SetUp_GselectIon(byte);
void SetUp_ModeOfSensorIon(byte);
void Ion_SwitchMode(byte mode);
byte AutoCalibration_Ion(void);
void Set_SelfTest(void);
void Clear_SelfTest(void); 
void ReadIonXOUTL(void); 
void ReadIonXOUTH(void); 
void ReadIonYOUTL(void); 
void ReadIonYOUTH(void); 
void ReadIonZOUTL(void); 
void ReadIonZOUTH(void); 
void ReadIonXOUT8(void); 
void ReadIonYOUT8(void);  
void ReadIonZOUT8(void); 
void ReadIonSTATUS(void); 
void ReadIonDETSR(void); 
void ReadIonTOUT(void); 
void ReadIonI2CAD(void);  
void ReadIonUSRINF(void); 
void ReadIonWHOAMI(void); 
void ReadIonXOFFL(void); 
void ReadIonXOFFH(void);  
void ReadIonYOFFL(void); 
void ReadIonYOFFH(void);  
void ReadIonZOFFL(void);  
void ReadIonZOFFH(void); 
void ReadIonINTRST(void); 
void ReadIonMCTL(void);  
void ReadIonCTL1(void);
void ReadIonCTL2(void); 
void ReadIonLDTH(void); 
void ReadIonPDTH(void); 
void ReadIonPW(void);  
void ReadIonLT(void); 
void ReadIonTW(void);                   


