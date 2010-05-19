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
* @version   1.0.0.0
* 
* @date      MAR-6-2008
* 
* @brief     ION (MMA7455L)
*                           Ion MMA7455L using SPI or IIC Communication Protocol 
*                           in Measure mode, Level and Pulse Detection (QE128)   
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
#define ION_AVDD_MASK 0x01
#define ION_DVDD_MASK 0x04 
#define ION_AVDD_INIT       PTDD &= ~ION_AVDD_MASK; PTDDD |= ION_AVDD_MASK;  
#define ION_AVDD_ON PTDD |= ION_AVDD_MASK;
#define ION_AVDD_OFF PTDD &= ~ION_AVDD_MASK;

#define ION_DVDD_INIT PTBD &= ~ION_DVDD_MASK; PTBDD |= ION_DVDD_MASK;                            
#define ION_DVDD_ON  PTBD |= ION_DVDD_MASK;
#define ION_DVDD_OFF PTBD &= ~ION_DVDD_MASK;

/* INTERCONNECT TO EVALUATION BOARD DEMOQE   //Reference document: Data Sheet MMA7455L Rev1, 11/2007

         ION                                DEMOQE                                                 
    
    1    DVDD                               PTB2 (21)               
              Digital Power for I/O (input) 

    2    GND                                GND (3)                
              Ground (input)

    3    N/C                                N/C (GND)          
              No connection or ground (input)

    4    IADDR0                             GND                
              I2C Address Bit 0 (input)

    5    GND                                GND                
              Ground (optional, input)

    6    AVDD                               PTD0 (29)        
              Analog Power (input)

    7    CS                                 PTGD1 (48)               
              SPI Enable (0, input)        
              I2C Enable (1, input)

    8    INT1/DRDY                          PTF0 (18)               
              Interrupt1/Data Ready (output)

    9    INT2                               PTF1 (20)              
              Interrupt2 (output) 

    10   N/C                                N/C (GND)          
              No connection or ground (input)

    11   Reserved                           GND                
              Connect to ground (input)

    12   SDO                                PTG0 (46)               
              SPI Serial Data Output (SDO, output) 
     
    13   SDA/SDI/SDO                        PTB6 (44)              
              I2C Serial Data (SDA, open drain)
              SPI Serial Data Input (SDI, input)
              3 Wire Serial Data Output (SDO, output)

    14   SCL/SPC                            PTB7 (42)                                                                                                                                                                
              I2C Serial Clock (SCL, input)
              SPI Serial Clock (SPC, input)                                                                           

    
    Note: MMA7455L does not include IIC pull up resistors for SDA and SCL. 
          Pull up from mcu (QE128) can be implemented together with 
          external pull up resistors (recommended, 4.7kOhms between SDA/SCL and DVDD).                                                  
                                                                                                                          */ 


// BUTTONS DECLARATIONS
#define BUTTON0 PTAD_PTAD2
#define BUTTON1 PTAD_PTAD3
#define BUTTON2 PTDD_PTDD2
#define BUTTON3 PTDD_PTDD3
#define INIT_BUTTON0  PTADD_PTADD2 = 0; PTAPE_PTAPE2 = 1; 
#define INIT_BUTTON1  PTADD_PTADD3 = 0; PTAPE_PTAPE3 = 1;
#define INIT_BUTTON2  PTDDD_PTDDD2 = 0; PTDPE_PTDPE2 = 1;
#define INIT_BUTTON3  PTDDD_PTDDD3 = 0; PTDPE_PTDPE3 = 1;
#define INIT_BUTTONS_MACRO INIT_BUTTON0; \
                           INIT_BUTTON1; \
                           INIT_BUTTON2; \
                           INIT_BUTTON3; 
                       
// ION INTERRUPT DECLARATION
#define INT1 PTFD_PTFD0      
#define INT1_D PTFDD_PTFDD0  
#define INT1_P PTFPE_PTFPE0
                                                          
#define INT2 PTFD_PTFD1      
#define INT2_D PTFDD_PTFDD1  
#define INT2_P PTFPE_PTFPE1

#define INIT_INT_MACRO INT1 = 0; \
                       INT1_D = INPUT; \
                       INT1_P = PULL_ON; \
                       INT2 = 0; \
                       INT2_D = INPUT; \
                       INT2_P = PULL_ON;
                       
//LED DEFINITION 	 
#define LED1 PTCD_PTCD0
#define LED2 PTCD_PTCD1
#define LED3 PTCD_PTCD2
#define LED4 PTCD_PTCD3
#define LED5 PTCD_PTCD4
#define LED6 PTCD_PTCD5
#define LED7 PTED_PTED6 
#define LED8 PTED_PTED7  
#define LED1_D PTCDD_PTCDD0
#define LED2_D PTCDD_PTCDD1
#define LED3_D PTCDD_PTCDD2
#define LED4_D PTCDD_PTCDD3
#define LED5_D PTCDD_PTCDD4
#define LED6_D PTCDD_PTCDD5
#define LED7_D PTEDD_PTEDD6 
#define LED8_D PTEDD_PTEDD7  
#define INIT_LED_MACRO LED1 = 1; \
                       LED1_D = 1; \
                       LED2 = 1; \
                       LED2_D = 1; \
                       LED3 = 1; \
                       LED3_D = 1; \
                       LED4 = 1; \
                       LED4_D = 1; \
                       LED5 = 1; \
                       LED5_D = 1; \
                       LED6 = 1; \
                       LED6_D = 1; \
                       LED7 = 1; \
                       LED7_D = 1; \
                       LED8 = 1; \
                       LED8_D = 1;

#define LED1_ON LED1=0;                   /* Set PTC0 Low (LED1 ON) */
#define LED2_ON LED2=0;                   /* Set PTC1 Low (LED2 ON) */
#define LED3_ON LED3=0;                   /* Set PTC2 Low (LED3 ON) */
#define LED4_ON LED4=0;                   /* Set PTC3 Low (LED4 ON) */
#define LED5_ON LED5=0;                   /* Set PTC4 Low (LED5 ON) */
#define LED6_ON LED6=0;                   /* Set PTC5 Low (LED6 ON) */
#define LED7_ON LED7=0;                   /* Set PTE6 Low (LED7 ON) */
#define LED8_ON LED8=0;                   /* Set PTE7 Low (LED8 ON) */

#define LED1_OFF LED1=1;                  /* Set PTC0 High (LED1 ON) */
#define LED2_OFF LED2=1;                  /* Set PTC1 High (LED2 ON) */
#define LED3_OFF LED3=1;                  /* Set PTC2 High (LED3 ON) */
#define LED4_OFF LED4=1;                  /* Set PTC3 High (LED4 ON) */
#define LED5_OFF LED5=1;                  /* Set PTC4 High (LED5 ON) */  
#define LED6_OFF LED6=1;                  /* Set PTC5 High (LED5 ON) */ 
#define LED7_OFF LED7=1;                  /* Set PTE6 High (LED7 ON) */
#define LED8_OFF LED8=1;                  /* Set PTE7 High (LED8 ON) */     
           

// ION MODES
#define ION_STANDBY 0
#define ION_MEASURE 1
#define ION_LEVEL_D 2
#define ION_PULSE_D 3
#define ION_DOUBLEPULSE_D 4

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
void SCI_Display(void);
void Delay(byte); 
void Delay_Xms(byte loop);
void Read_AccelerometerIon(void);
void Init_SensorIon(void);
void SetUp_GselectIon(byte gsel);
void SetUp_ModeOfSensorIon(byte *data);
void Change_SensorInterface(void);
void Ion_SwitchMode(byte mode);
void Ion_ClearIntLatch(void);
void Ion_ClearIntLatchINT1(void);
void Ion_ClearIntLatchINT2(void);
byte AutoCalibration_Ion(void);
void Dummy_Pointer(byte dum1, void *dummy, byte dum2);
void ReadAllIonRegisters(void);
void initTPM1CH0(unsigned int,unsigned int,unsigned int);
void initTPM2CH0(unsigned int,unsigned int,unsigned int);
void Reset_Counter1(void);
void Reset_Counter2(void);
void Clear_TOF1(void); 
void Clear_TOF2(void);
void Clear_CHOF1(void);
void Clear_CHOF2(void);
void SetUp_TPM_Value1(unsigned int,unsigned int,unsigned int);
void SetUp_TPM_Value2(unsigned int,unsigned int,unsigned int);
void Set_SelfTest(void);
void Clear_SelfTest(void);
void ReadIonMODE(void); 
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
