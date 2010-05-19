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
*                           Ion MMA7455L using SPI or IIC Communication Protocol 
*                           in Measure mode, Level and Pulse Detection (QE128)  
*
*******************************************************************************
*
* 
*
******************************************************************************/

#include "derivative.h" /* include peripheral declarations */
#include "doonstack.h"
#include "main.h"
#include "my_types.h"
#include "mcu_init.h"
#include "adc.h"
#include "sci.h"
#include "sw_spi.h"
#include "sw_iic.h"



#pragma DATA_SEG SHORT _DATA_ZEROPAGE
unsigned long counterreset; 
byte n;
byte X,Y,Z;
byte ion_reg[32];
signed int calib[3];
byte break_loop0,break_loop1,break_loop2,break_loop3,break_loop4;  
byte tmp_buffer[10];
unsigned char init;       
unsigned char readTPM1C0SC;
unsigned long counterVALUE1;
unsigned char counterVALUE1L;
unsigned char counterVALUE1H;
unsigned char readTPM2C0SC;
unsigned long counterVALUE2;
unsigned char counterVALUE2L;
unsigned char counterVALUE2H;
byte  temp_QE128H,temp_QE128L,VDD_QE128H,VDD_QE128L;
byte x_hermesH,x_hermesL,y_hermesH,y_hermesL,z_hermesH,z_hermesL;    

byte mode = ION_MEASURE;

byte iic_active = 1;  //IIC (select "1") or SPI (select "0")  

void (*p_Ion_InitInterface)(void) = SwIIC_Init;
byte (*p_Ion_ReadByte)(byte) = SwIIC_ReadByte;
void (*p_Ion_WriteByte)(byte, byte) = SwIIC_WriteByte;
void (*p_Ion_MultiReadBytes)(byte, byte*, byte) = SwIIC_MultiReadBytes;
void (*p_Ion_MultiWriteBytes)(byte, byte*, byte) = SwIIC_MultiWriteBytes;

#pragma DATA_SEG DEFAULT

READ_MODE ReadMODE;
READ_REGXOUTL ReadRegXOUTL;              //Review XOUTL in Real-Time Debugger
READ_REGXOUTH ReadRegXOUTH;              //Review XOUTH in Real-Time Debugger
READ_REGYOUTL ReadRegYOUTL;              //Review YOUTL in Real-Time Debugger
READ_REGYOUTH ReadRegYOUTH;              //Review YOUTH in Real-Time Debugger
READ_REGZOUTL ReadRegZOUTL;              //Review ZOUTL in Real-Time Debugger
READ_REGZOUTH ReadRegZOUTH;              //Review ZOUTH in Real-Time Debugger
READ_REGXOUT8 ReadRegXOUT8;              //Review XOUT8 in Real-Time Debugger
READ_REGYOUT8 ReadRegYOUT8;              //Review YOUT8 in Real-Time Debugger
READ_REGZOUT8 ReadRegZOUT8;              //Review ZOUT8 in Real-Time Debugger
READ_REGSTATUS ReadRegSTATUS;            //Review STATUS in Real-Time Debugger
READ_REGDETSR ReadRegDETSRC;             //Review DETSRC in Real-Time Debugger
READ_REGTOUT ReadRegTOUT;                //Review TOUT in Real-Time Debugger
READ_REGRESERVED1 ReadRegRESERVED1;      //Review RESERVED1 in Real-Time Debugger
READ_REGI2CAD ReadRegI2CAD;              //Review I2CAD in Real-Time Debugger
READ_REGUSRINF ReadRegUSRINF;            //Review USRINF in Real-Time Debugger
READ_REGWHOAMI ReadRegWHOAMI;            //Review WHOAMI in Real-Time Debugger
READ_REGXOFFL ReadRegXOFFL;              //Review XOFFL in Real-Time Debugger
READ_REGXOFFH ReadRegXOFFH;              //Review XOFFH in Real-Time Debugger
READ_REGYOFFL ReadRegYOFFL;              //Review YOFFL in Real-Time Debugger
READ_REGYOFFH ReadRegYOFFH;              //Review YOFFH in Real-Time Debugger
READ_REGZOFFL ReadRegZOFFL;              //Review ZOFFL in Real-Time Debugger
READ_REGZOFFH ReadRegZOFFH;              //Review ZOFFH in Real-Time Debugger
READ_REGINTRST ReadRegINTRST;            //Review INTRST in Real-Time Debugger
READ_REGMCTL ReadRegMCTL;                //Review MCTL in Real-Time Debugger
READ_REGCTL1 ReadRegCTL1;                //Review CTL1 in Real-Time Debugger
READ_REGCTL2 ReadRegCTL2;                //Review CTL2 in Real-Time Debugger
READ_REGLDTH ReadRegLDTH;                //Review LDTH in Real-Time Debugger
READ_REGPDTH ReadRegPDTH;                //Review PDTH in Real-Time Debugger
READ_REGPW ReadRegPW;                    //Review PW in Real-Time Debugger
READ_REGLT ReadRegLT;                    //Review LT in Real-Time Debugger
READ_REGTW ReadRegTW;                    //Review TW in Real-Time Debugger
READ_REGRESERVED2 ReadRegRESERVED2;      //Review RESERVED2 in Real-Time Debugger

#pragma CONST_SEG EEPROM       
const signed int calib_eeprom[3];                     





/**************************************************************
*	Function: 	Dummy_Pointer
*	Parameters: dummy pointer to void
*	Return:		none
*
*	Dummy function
**************************************************************/
#pragma MESSAGE DISABLE C5703   /* 
                                 * Warning C5703: Not used declared parameter
                                 */
 
void Dummy_Pointer(byte dum1, void *dummy, byte dum2)
{

}

/**************************************************************
*	Function: 	CheckButt0 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Specific routine execution if button pressed 
**************************************************************/
void CheckButt0(void)
{
  static byte last_st = 0;
  
  if(BUTTON0 != last_st)
  {
    if(BUTTON0 == 0)
    {
      LED1_ON;
      Delay_Xms(50);
      LED1_OFF;
      
      break_loop0=1;
      break_loop1=1;
      break_loop2=1;
      break_loop3=1;
      break_loop4=1;
      
      Delay(200);
      
      //(void)AutoCalibration_Ion();
      
      /* ION IN STANDBY MODE */
      /***********************/
      //tmp_buffer[0] = ION_STANDBY;   
      //SetUp_ModeOfSensorIon(&tmp_buffer[0]);
      
      /* ION IN MEASURE MODE */
      /***********************/
      SetUp_GselectIon(ION_G2);
      tmp_buffer[0] = ION_MEASURE;   // Define measure mode
      /* Set up MCTL */
      tmp_buffer[1] = 0x00;          // DRDP  0 Data ready status is output to INT1/DRDY pin 
      /* Set up CTL1 */
      //tmp_buffer[2] = 0b10000000;  // DFBW  1 Digital filter band width is  125Hz
      tmp_buffer[2] = 0b00000000;
      /* Set up CTL2 */  
      tmp_buffer[3] = 0x00;          // LDPOL OR        
      tmp_buffer[4] = 0x00;          // PDPOL OR
      /* Set up LDTH */          
      tmp_buffer[5] = 0x00;          // LDTH is 3g
      /* Set up PDTH */
      tmp_buffer[6] = 0x00;          // PDTH is 5g
      /* Set up PW */         
      tmp_buffer[7] = 0x00;          // 1LSB is 0.5ms => 4ms   
      /* Set up LT */
      tmp_buffer[8] = 0x00;          // Pulse every 232ms 
      /* Set up TW */           
      tmp_buffer[9] = 0x00;                           
      SetUp_ModeOfSensorIon(&tmp_buffer[0]);
      
      Delay_Xms(50); 
 
    }
  }
  last_st = BUTTON0;
}


/**************************************************************
*	Function: 	CheckButt1 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Specific routine execution if button pressed 
**************************************************************/
void CheckButt1(void)
{
  static byte last_st = 0;
  
  if(BUTTON1 != last_st)
  {
    if(BUTTON1 == 0)
    {
      LED2_ON;
      Delay_Xms(50);
      LED2_OFF;
       
      break_loop0=1;
      break_loop1=1;
      break_loop2=1;
      break_loop3=1;
      break_loop4=1;
      
      Delay(200); 
      
      /* ION IN LEVEL DETECTION MODE */
      /*******************************/
      tmp_buffer[0] = ION_LEVEL_D; // Level detection mode
      /* Set up MCTL */
      tmp_buffer[1] = 0x01;        // DRDP  1 Data ready status is not output to INT1/DRDY pin              
      /* Set up CTL1 */                                                   
      tmp_buffer[2] = 0b10011000;  // DFBW  1  Digital filter band width is 125Hz
                                   // THOPT 0  ABSOLUTE, LDTH is 7bits unsigned value
                                   // ZDA   0  Z axis is enabled for detection
                                   // YDA   1  Y axis is disabled for detection  
                                   // XDA   1  X axis is disabled for detection
                                   //       00 Level detection    
                                   //              INT1:Level Detection (used)
                                   //             INT2:Pulse Detection (not used)
                                   //       0   INT1 register is routed to INT1 pin 
      /* Set up CTL2 */  
      tmp_buffer[3] = 0x00;        // LDPOL OR        
      tmp_buffer[4] = 0x00;        // PDPOL OR
      /* Set up LDTH */          
      tmp_buffer[5] = 0x30;        // LDTH is 3g
      //tmp_buffer[5] = 0x20;        // LDTH is 5g
      /* Set up PDTH */
      tmp_buffer[6] = 0x00;
      /* Set up PW */         
      tmp_buffer[7] = 0x00;                   
      /* Set up LT */
      tmp_buffer[8] = 0x00;   
      /* Set up TW */           
      tmp_buffer[9] = 0x00;   
      SetUp_ModeOfSensorIon(&tmp_buffer[0]);
         
      Delay_Xms(50); 
       
    }
  }
  last_st = BUTTON1;
}

/**************************************************************
*	Function:   CheckButt2	 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Specific routine execution if button pressed 
**************************************************************/

void CheckButt2(void)
{
  static byte last_st = 0;
  
  if(BUTTON2 != last_st)
  {
    if(BUTTON2 == 0)
    {
      LED3_ON;
      Delay_Xms(50);
      LED3_OFF;
       
      break_loop0=1;
      break_loop1=1;
      break_loop2=1;
      break_loop3=1;
      break_loop4=1;
      
      Delay(200);
      
      /* ION IN LEVEL & PULSE DETECTION MODE */
      /**************************************/
       tmp_buffer[0] = ION_PULSE_D; // Pulse detection mode
       /* Set up MCTL */
       tmp_buffer[1] = 0x01;        // DRDP  1 Data ready status is not output to INT1/DRDY pin       
       /* Set up CTL1 */                                                   
       tmp_buffer[2] = 0b10011000;  // DFBW  1  Digital filter band width is 125Hz
                                    // THOPT 0  ABSOLUTE, LDTH is 7bits unsigned value
                                    // ZDA   0  Z axis is enabled for detection
                                    // YDA   1  Y axis is disabled for detection  
                                    // XDA   1  X axis is disabled for detection
                                    // INTREG[1:0]   00    INT1:Level Detection 
                                    //                     INT2:Pulse Detection
                                    //       0  INT1 pin is routed to INT1 register
       /* Set up CTL2 */  
       tmp_buffer[3] = 0x00;        // LDPOL OR
       tmp_buffer[4] = 0x00;        // PDPOL OR
       /* Set up LDTH */          
       tmp_buffer[5] = 0x30;        // LDTH is 3g
       /* Set up PDTH */
       tmp_buffer[6] = 0x50;        // PDTH is 5g
       /* Set up PW */         
       tmp_buffer[7] = 0x08;        // 1LSB is 0.5ms => 4ms   
       /* Set up LT */
       tmp_buffer[8] = 0xD5;        // Pulse every 213ms 
       /* Set up TW */             
       tmp_buffer[9] = 0x0A;                               
       SetUp_ModeOfSensorIon(&tmp_buffer[0]);
       
       Delay_Xms(50);  
      
    }
  }
  last_st = BUTTON2;
}

/**************************************************************
*	Function: 	CheckButt3 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Specific routine execution if button pressed 
**************************************************************/

void CheckButt3(void)
{
  static byte last_st = 0;
  
  if(BUTTON3 != last_st)
  {
    if(BUTTON3 == 0)
    {
      LED4_ON;
      Delay_Xms(50);
      LED4_OFF;
     
      break_loop0=1;
      break_loop1=1;
      break_loop2=1;
      break_loop3=1;
      break_loop4=1;
      
       Delay(200);
      
       // ION IN DOUBLE PULSE DETECTION MODE 
       tmp_buffer[0] = ION_DOUBLEPULSE_D;   // Pulse detection mode
       // Set up MCTL 
       tmp_buffer[1] = 0x01;                // DRDP  1 Data ready status is not output to INT1/DRDY pin
       // Set up CTL1 
       tmp_buffer[2] = 0b10011100;          // DFBW   1  Digital filter band width is 125Hz
                                            // THOPT  0  ABSOLUTE, LDTH is 7bits unsigned value
                                            // ZDA    0  Z axis is enabled for detection
                                            // YDA    1  Y axis is disabled for detection  
                                            // XDA    1  X axis is disabled for detection
                                            //       10     INT1:Single Pulse Detection 
                                            //              INT2:Double Pulse Detection
                                            //       0  INT1 pin is routed to INT1 register
       // Set up CTL2  
       tmp_buffer[3] = 0x00;                // DPOL=0 (0R)
       tmp_buffer[4] = 0x00;                // PDPL=0 (0R) 
       // Set up LDTH          
       tmp_buffer[5] = 0x80;                                                            
       // Set up PDTH  
       tmp_buffer[6] = 0x50;                // PDTH is 5g 
       //tmp_buffer[6] = 0x20;                // PDTH is 2g                                               
       // Set up PW 
       tmp_buffer[7] = 0x08;                // 1LSB is 0.5ms =>  4ms                  
       // Set up LT            
       tmp_buffer[8] = 0x0F;                // 1LSB is   1ms =>  4ms
       // Set up TW  
       tmp_buffer[9] = 0x0A;                // 1LSB is   1ms => 10ms                         
       SetUp_ModeOfSensorIon(&tmp_buffer[0]);   
       
       Delay_Xms(50); 
    }
  }
  last_st = BUTTON3;
}

/**************************************************************
*	Function: 	main 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Interface with Ion (MMA7455L) using SPI or IIC 
*             and transfer data in SCI 
**************************************************************/
void main(void)
{
    //MCU init
    mcu_init();
  	//ADC init
    initADC_Bits(ADC_Def); // ADC 8Bits, 10Bits or 12Bits for reading analog output of Hermes
    //SCI init	
  	SCI_Init();
  	INIT_LED_MACRO
    INIT_BUTTONS_MACRO
    Delay_Xms(50);
    LED1_ON;
    
    //ION init
    ION_DVDD_INIT;
    ION_AVDD_INIT;
    ION_DVDD_ON;
    ION_AVDD_ON;
    Delay_Xms(50);
    LED2_ON;
    
    // Select IIC or SPI communication
    Change_SensorInterface();
    // Init Ion (measure mode 2g selected)
    Init_SensorIon();
  	Delay_Xms(50);
    LED3_ON;
    
    // Initialization flash memory (store calibration data)
    FCDIV = FCDIV_PRDIV8_MASK | (17 & FCDIV_FDIV_MASK);      // 28 544 700 / (8*(18+1))=188kHz
                                                             // 28 544 700 / (8*(17+1))=198kHz                                                  
    // Load previous calibration data (Ion MMA7455L does not keep calibration data)    
    for(n=0;n<3;n++)
    {    	    
 	  p_Ion_WriteByte(XOFFL + n*2, (byte) (((word)calib_eeprom[n]) % 256));
 	  p_Ion_WriteByte(XOFFH + n*2, (byte) (((word)calib_eeprom[n]) / 256)); 	  
 	  calib[n] = calib_eeprom[n];
 	  }

    
    //Calibration of Ion MMA7455L (device horizontal to read 0g on X and Y with +1g on Z axis)
    (void)AutoCalibration_Ion();   //To be used only if new calibration is required  
    
    Delay_Xms(50);
    LED4_ON;
  
    Delay_Xms(100);
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    
    initTemp_QE128();        //REG_VDD used (external power supply)
    
    //Read Ion Registers (view in Real-Time Debugger)
    ReadAllIonRegisters(); 

    init=0;
    counterreset=0;              
 
   for(;;)
   {  
      
  	  if(tmp_buffer[0] == ION_STANDBY)
      {
      
       break_loop0=0;
      
       while(break_loop0==0) 
       {
          LED1_ON;
          LED2_ON;
          LED3_ON;
          LED4_ON;
        
          CheckButt0(); 
          CheckButt1();
          CheckButt2();
          CheckButt3();
          
          LED1_OFF;
          LED2_OFF;
          LED3_OFF;
          LED4_OFF;
       }
      }
    
    
      if(tmp_buffer[0] == ION_MEASURE)      //Selected through "Check_Butt0"
      {
      
       break_loop1=0;
      
       while(break_loop1==0) 
       {
          LED1_ON;

          // Temperature Measurement QE128
  	      Temp_QE128();       //ADC 12Bits used

          // VDD Measurement QE128  
          VDD_QE128();        //ADC 12Bits used
         
  	      // Acceleration Analog Output 
  	      ReadADC_Hermes();  //To be used to display Hermes MMA7260QT available in DemoQE
  	     
  	      Read_AccelerometerIon();
  	
  	      SCI_Display();
        
          CheckButt0(); 
          CheckButt1();
          CheckButt2();
          CheckButt3();
          
          LED1_OFF;        
       }
      }
      
      
      if(tmp_buffer[0] == ION_LEVEL_D)       //Selected through "Check_Butt1"
      {
      
       break_loop2=0;
       
       while(break_loop2==0) 
       {
          LED2_ON; 
          
          if(INT1) Ion_ClearIntLatchINT1();  // (INT1 register routed to INT1 pin) 

          CheckButt0();            
          CheckButt1();            
          CheckButt2();
          CheckButt3();
                                    
          LED2_OFF;
       }
      }


      if(tmp_buffer[0] == ION_PULSE_D)       //Selected through "Check_Butt2"
      {
       /*Init counter for ION Double Pulse Detection Reset*/
       /*=================================================*/ 
  	   initTPM1CH0(tmp_buffer[7],tmp_buffer[8],tmp_buffer[9]);
  	     
       /* Reset counter */
       /*==============*/
       Reset_Counter1();
       Clear_TOF1();    
       Clear_CHOF1();
      
       break_loop3=0;
       counterreset=0;
       
       while(break_loop3==0) 
       {
          LED3_ON;                 
          
          if(INT1) Ion_ClearIntLatchINT1(); // Level detection
                                            // (INT1 register routed to INT1 pin)
          if(INT2) 
          {
                Ion_ClearIntLatchINT2();    // Pulse detection (single or double)
                                            // (INT2 register routed to INT2 pin)
                /* Reset counter */
                /*===============*/
                Reset_Counter1();
                Clear_TOF1();    
                Clear_CHOF1();
          }

          /* Ion Automatic Reset: STOP OF COUNTER */  //Double pulse detection
          /*======================================*/   
          readTPM1C0SC=TPM1C0SC;
          if(readTPM1C0SC==0x90) 
          {                             
              Delay_Xms(1);
              Ion_ClearIntLatch();           //Clear of interrupt is forced to reset detection cycle in case of no detect
              counterreset++;
              /* Reset counter */
              /*===============*/
              Reset_Counter1();
              Clear_TOF1();    
              Clear_CHOF1();                   
          } 

          CheckButt0();             
          CheckButt1();           
          CheckButt2();
          CheckButt3();

          LED3_OFF;  
       }
      }
      
      if(tmp_buffer[0] == ION_DOUBLEPULSE_D)     //Selected through "Check_Butt3"
      {
          /* Init counter for ION Double Pulse Detection Reset */ 
  	      /*===================================================*/
  	      initTPM2CH0(tmp_buffer[7],tmp_buffer[8],tmp_buffer[9]);
  	     
          /* Reset counter */
          /*===============*/
          Reset_Counter2();
          Clear_TOF2();    
          Clear_CHOF2();
                    
          break_loop4=0;
          init=0;
          counterreset=0;
          
          while(break_loop4==0) 
          { 
          LED4_ON;
          
           if(INT1&&!init)                             //Single pulse detection
           {     
              init=1;
              /* Ion Automatic Reset: DETECTION START OF COUNTER */ 
              /*=================================================*/        
              Reset_Counter2();
              Clear_TOF2();    
              Clear_CHOF2();   
           }
          
           if(INT2)                                    //Double pulse detection
           {
              init=0;
              Ion_ClearIntLatch();
           }                                    
              
          /* Ion Automatic Reset: STOP OF COUNTER */  //Double pulse detection
          /*======================================*/
           readTPM2C0SC=TPM2C0SC;
           if(readTPM2C0SC==0x90) 
           {
              counterreset++;
              init=0;
              
              Ion_ClearIntLatch();           //Clear of interrupt is forced to reset detection cycle in case of no detect                    
                                  
              /* Reset counter */
              /*===============*/
              Reset_Counter2();
              Clear_TOF2();          
              Clear_CHOF2();        
           } 
           
          CheckButt0(); 
          CheckButt1();
          CheckButt2();
          CheckButt3();
 
          LED4_OFF;
          }
      }
   }
}

/**************************************************************
*	Function: 	Delay
*	Parameters: 16-bit delay value
*	Return:		none
*
*	Simple delay loop.
**************************************************************/
void Delay(byte count) 
{
	byte i;
	for (i=0; i<count; i++)
	{
    //asm nop; 
 }
}

/**************************************************************
*	Function: 	SCI_Display 
*	Parameters: None 
*	Return:		  None
*
*	Note:       SCI transfer to GUI
**************************************************************/
void SCI_Display(void) 
{
  
  	  /*Parameter1 */
      SCI_TxByte(0xFF); //FF00 identifies X axis acceleration (8Bits)
      SCI_TxByte(0x00);
      SCI_TxByte(0x00);
      SCI_TxByte(X);
    
      /*Parameter2 */
      SCI_TxByte(0xFE); //FE01 identifies Y axis acceleration (8Bits)
      SCI_TxByte(0x01);
      SCI_TxByte(0x00);
      SCI_TxByte(Y);
    
      /*Parameter3 */
      SCI_TxByte(0xFD); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x02);
      SCI_TxByte(0x00);
      SCI_TxByte(Z);
      
      /*Parameter4 */
      SCI_TxByte(0xFC); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x03);
      SCI_TxByte(VDD_QE128H);
      SCI_TxByte(VDD_QE128L);
      
      /*Parameter5 */
      SCI_TxByte(0xFB);
      SCI_TxByte(0x04);
      SCI_TxByte(x_hermesH);
      SCI_TxByte(x_hermesL);
      
      /*Parameter6 */
      SCI_TxByte(0xFA); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x05);
      SCI_TxByte(y_hermesH);
      SCI_TxByte(y_hermesL);
      
      /*Parameter7 */
      SCI_TxByte(0xF9); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x06);
      SCI_TxByte(z_hermesH);
      SCI_TxByte(z_hermesL);
      
      /*Parameter8 */
      SCI_TxByte(0xF8); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x07);
      SCI_TxByte(temp_QE128H);
      SCI_TxByte(temp_QE128L);
      
}

/**************************************************************
*	Function: 	Delay_Xms 
*	Parameters: loop  
*	Return:		  None
*
*	Note:       Delay in ms defined by loop
*             (Bus Clock: 20MHz)
**************************************************************/
void Delay_Xms(byte loop)
{
  byte i,j,k;
  
  for(i=0;i<=loop;i++)
  {
    for(k=0;k<10;k++)
    {
      for(j=0;j<200;j++)        /* Delay_Xms(50) is approximately 50ms */
      {
       asm nop;
      }
    }  
  }
}

/**************************************************************
*	Function: 	Read_AccelerometerIon 
*	Parameters: None 
*	Return:		  Acceleration (8Bits) for X, Y and Z
*
* Note:       Acceleration data is signed byte (2's compliment) 
*             Conversion signed (2's compliment) to unsigned 
*             implemented.            	
**************************************************************/
void Read_AccelerometerIon(void)
{
  byte tmpbuff[3];
  
  	if(iic_active)
  	{
  	while(!INT1); // wait for valid new data on DRDY signal
    p_Ion_MultiReadBytes(XOUT8,tmpbuff,3); // read 8 bits output values from ION      	 	  
  	} 
  	else
  	{
  	while(!INT1); // wait for valid new data on DRDY signal
    tmpbuff[0] = p_Ion_ReadByte(XOUT8);                              
    tmpbuff[1] = p_Ion_ReadByte(YOUT8);                            
    tmpbuff[2] = p_Ion_ReadByte(ZOUT8);                          
  	}
  	
  	ReadRegXOUT8.MergedBits.Reg=tmpbuff[0];  
  	ReadRegYOUT8.MergedBits.Reg=tmpbuff[1];
  	ReadRegZOUT8.MergedBits.Reg=tmpbuff[2];
  	
    X = tmpbuff[0]+127;  //conversion signed to unsigned byte           
    Y = tmpbuff[1]+127;  //conversion signed to unsigned byte        
    Z = tmpbuff[2]+127;  //conversion signed to unsigned byte     
   
 }
 
/**************************************************************
*	Function: 	Init_SensorIon
*	Parameters: none
*	Return:		none
*
*	Function Initialize sensor - ION version
**************************************************************/ 
void Init_SensorIon(void)
{
	p_Ion_InitInterface();         //Initialization of communication with ION sensor
	INIT_INT_MACRO                 // IIC (iic_active=1)  
	Delay_Xms(20);                 // SPI (iic_active=0) 
	
	p_Ion_WriteByte(MCTL,0x05);    //DRDP=0  (data ready is output on to INT1/DRDY pin)
	ReadIonMCTL();                 //SPI3W=0 (SPI is 4 wire mode)
	                               //STON=0  (self test is not enabled)
	                               //GLVL[1:0]=01 (2g range) 
	                               //MODE[1:0]=01 (measurement mode)
	
	
	p_Ion_WriteByte(CTL1,0x00);    //This is to set up DFBW: 62.5Hz for data rate 125Hz 
	//p_Ion_WriteByte(CTL1,0x80);  //This is to set up DFBW:  125Hz for data rate 250Hz                
  ReadIonCTL1();              
	
	tmp_buffer[0] = ION_MEASURE; // for measure  
	
	Ion_ClearIntLatch();
	mode = ION_MEASURE;
	ReadIonMODE();        
}

/**************************************************************
*	Function: 	SetUp_GselectIon
*	Parameters: value of g_select
*	Return:		none
*
*	Function Initialize sensor - ION version
**************************************************************/ 
void SetUp_GselectIon(byte gsel)
{
	byte i;
	
	i = p_Ion_ReadByte(MCTL);
	i &= ~0x0c;
	if(gsel > 2) gsel = 2;
	i |= (gsel << 2);
	p_Ion_WriteByte(MCTL,i);
	ReadIonMCTL();	
}

/**************************************************************
*	Function: 	SetUp_ModeOfSensorIon
*	Parameters: pointer to data buffer
*	Return:		none
*
*	Function set up right mode of sensor - ION version
**************************************************************/ 
void SetUp_ModeOfSensorIon(byte *data)
{
byte k;

        /* Set up of Ion Registers for standby, measure, level and/or pulse */ 
        /********************************************************************/
        /*
        Set up MCTL
	      Bit[7]   -                        
        Bit[6]   DRDP       0 Data ready status is output to INT1/DRDY
                            1 Data ready status is not output to INT1/DRDY   
        Bit[3]   SPI3W      0 SPI is 4 wire mode 
                            1 SPI is 3 wire mode
        Bit[4]   STON       0 Self test is not enabled                   
                            1 Self test is enabled                           
        Bit[3:2] GLVL[1:0]  00 8g selected for measurement mode, detection is 8g range                    
                            01 2g selected for measurement mode, detection is 8g range 
                            10 4g selected for measurement mode, detection is 8g range                            
        Bit[1:0] MOD[1:0]   00 Standby Mode 
                            01 Measurement Mode
                            10 Level Detection Mode
                            11 Pulse Detection Mode        
                            
        Set up CTL1 
	      Bit[7]  DFBW        0 Digital filter band width is 62.5Hz                                                       
                            1 Digital filter band width is  125Hz                
        Bit[6]  THOPT       0 ABSOLUTE, LDTH is 7bits unsigned value
                            1 Pos./Neg., LDTH is 8bits signed value
                            Note: THOPT applies only to level detection   
        Bit[5]  ZDA         0 Z axis is enabled for detection                   
                            1 Z axis is disabled for detection                            
        Bit[4]  YDA         0 Y axis is enabled for detection                   
                            1 Y axis is disabled for detection                            
        Bit[3]  XDA         0 X axis is enabled for detection                   
                            1 X axis is disabled for detection                            
        Bit[2:1] INTGR[1:0] 00 INT1: Level detection / INT2: Pulse detection                             
                            01 INT1: Pulse detection / INT2: Level detection                        
                            10 INT1: Single Pulse detection / INT2: Pulse detection                                                           
        Bit[0]  INTPIN      0 INT1 is routed to INT1                            
                            1 INT1 is routed to INT2       
                            
        Set up CTL2 
        Bit[7:3] -
        Bit[2]   DRDV0 Drive strength SDA/SDO
        Bit[1]   PDPL  Pulse detection
        Bit[0]   LDPL  Motion detection
                       LDPL=0 Motion detection
                       LDPL=1 Free Fall               
                                                                                        */


	  Ion_ClearIntLatch();  // clear ION latches
	  
	  
	  switch(data[0])
	  {
	  
	   case ION_STANDBY:
	   Ion_SwitchMode(ION_STANDBY);
	   mode = ION_STANDBY;
	   ReadIonMODE();
	   break;
	   
	    
	   case ION_MEASURE: // Measure mode
	   Ion_SwitchMode(ION_MEASURE);
     /* Set up MCTL */
     if(tmp_buffer[1]==0x01) 
     {
     k = p_Ion_ReadByte(MCTL);
	   k |= 0x40;               //Force DRDP at 1: 1 Data ready status is not output to INT1/DRDY  
	   p_Ion_WriteByte(MCTL,k);
     } 
     else
     {
     k = p_Ion_ReadByte(MCTL);                                              
	   k &= 0xBF;               //Force DRDP at 0: Data ready status is output to INT1/DRDY pin
	   p_Ion_WriteByte(MCTL,k);
     }
     /* Set up CTL1 */                                                        
	   p_Ion_WriteByte(CTL1,tmp_buffer[2]);            

	                                                                                     
  	 mode = ION_MEASURE;
  	 ReadIonMODE();
  	 ReadAllIonRegisters(); 
  	 Ion_ClearIntLatch();  // clear ION latches
	   break;


	   case ION_LEVEL_D: // Level detection mode
	   Ion_SwitchMode(ION_LEVEL_D); 
	   /* Set up MCTL */
     if(tmp_buffer[1]==0x01) 
     {
     k = p_Ion_ReadByte(MCTL);
	   k |= 0x40;               //Force DRDP at 1: 1 Data ready status is not output to INT1/DRDY  
	   p_Ion_WriteByte(MCTL,k);
     } 
     else
     {
     k = p_Ion_ReadByte(MCTL);                                              
	   k &= 0xBF;               //Force DRDP at 0: Data ready status is output to INT1/DRDY pin
	   p_Ion_WriteByte(MCTL,k);
     }
	   /* Set up CTL1 */                                
	   p_Ion_WriteByte(CTL1,tmp_buffer[2]);
     /* Set up CTL2 */ 
	   k = p_Ion_ReadByte(CTL2);
	   if(tmp_buffer[3]==0x01) 
	   {
	    k|=0x01;   /* Case LDPOL=1*/
	   } 
	   else
	   {
     k&=0xFE;    /* Case LDPOL=0*/                 
	   }
     p_Ion_WriteByte(CTL2,k);
     if(iic_active==1)
     {
     p_Ion_MultiWriteBytes(LDTH,&tmp_buffer[5],5);
     }
     else
     { 
     /* Set up LDTH */
     p_Ion_WriteByte(LDTH,tmp_buffer[5]);
  	 /* Set up PDTH */
  	 p_Ion_WriteByte(PDTH,tmp_buffer[6]);
  	 /* Set up PW */
  	 p_Ion_WriteByte(PW,tmp_buffer[7]);
  	 /* Set up LT */ 
  	 p_Ion_WriteByte(LT,tmp_buffer[8]);
  	 /* Set up TW */
  	 p_Ion_WriteByte(TW,tmp_buffer[9]);
     }
     ReadIonLDTH();
     ReadIonPDTH();
     ReadIonPW();
     ReadIonLT();
     ReadIonTW();
	                    
     mode = ION_LEVEL_D;
     ReadIonMODE();
     ReadAllIonRegisters(); 
     Ion_ClearIntLatch();  // clear ION latches
     break;
    
    
     case ION_PULSE_D: // Pulse detection mode
     /* Set up MCTL */  
     if(tmp_buffer[1]==0x01) 
     {
     k = p_Ion_ReadByte(MCTL);
	   k |= 0x40;               //Force DRDP at 1: 1 Data ready status is not output to INT1/DRDY  
	   p_Ion_WriteByte(MCTL,k);
     } 
     else
     {
     k = p_Ion_ReadByte(MCTL);                                              
	   k &= 0xBF;               //Force DRDP at 0: Data ready status is output to INT1/DRDY pin
	   p_Ion_WriteByte(MCTL,k);
     }
     /* Set up CTL1 */  
     p_Ion_WriteByte(CTL1,tmp_buffer[2]);
     /* Set up CTL2 */ 
	   k = p_Ion_ReadByte(CTL2);
	   if(tmp_buffer[3]==0x01) 
	   {
	    k|=0x01;   /* Case LDPOL=1*/
	   } 
	   else
	   {
     k&=0xFE;    /* Case LDPOL=0*/                 
	   }
	   if(tmp_buffer[4]==0x01)
	   {
	    k|=0x02;   /* Case PDPOL=1*/
	   } 
	   else
	   {
     k&=0xFD;    /* Case PDPOL=0*/                 
	   }
     p_Ion_WriteByte(CTL2,k);
     if(iic_active==1)
     {
     p_Ion_MultiWriteBytes(LDTH,&tmp_buffer[5],5);
     }
     else
     { 
     /* Set up LDTH */
     p_Ion_WriteByte(LDTH,tmp_buffer[5]);
  	 /* Set up PDTH */
  	 p_Ion_WriteByte(PDTH,tmp_buffer[6]);
  	 /* Set up PW */
  	 p_Ion_WriteByte(PW,tmp_buffer[7]);
  	 /* Set up LT */ 
  	 p_Ion_WriteByte(LT,tmp_buffer[8]);
  	 /* Set up TW */
  	 p_Ion_WriteByte(TW,tmp_buffer[9]);
     }
     ReadIonLDTH();
     ReadIonPDTH();
     ReadIonPW();
     ReadIonLT();
     ReadIonTW();
     
     Ion_SwitchMode(ION_PULSE_D);         
     mode = ION_PULSE_D;
     ReadIonMODE();
     ReadAllIonRegisters();
     Ion_ClearIntLatch();  // clear ION latches  
     break;
    
    
     case ION_DOUBLEPULSE_D: // Pulse detection mode
 
     /* Set up MCTL */  
     if(tmp_buffer[1]==0x01) 
     {
     k = p_Ion_ReadByte(MCTL);
	   k |= 0x40;               //Force DRDP at 1: 1 Data ready status is not output to INT1/DRDY  
	   p_Ion_WriteByte(MCTL,k);
     } 
     else
     {
     k = p_Ion_ReadByte(MCTL);                                              
	   k &= 0xBF;               //Force DRDP at 0: Data ready status is output to INT1/DRDY pin
	   p_Ion_WriteByte(MCTL,k);
     }
     /* Set up CTL1 */  
     p_Ion_WriteByte(CTL1,tmp_buffer[2]);
     /* Set up CTL2 */ 
	   k = p_Ion_ReadByte(CTL2);
	   if(tmp_buffer[3]==0x01) 
	   {
	    k|=0x01;   /* Case LDPOL=1*/
	   } 
	   else
	   {
     k&=0xFE;    /* Case LDPOL=0*/                 
	   }
	   if(tmp_buffer[4]==0x01)
	   {
	    k|=0x02;   /* Case PDPOL=1*/
	   } 
	   else
	   {
     k&=0xFD;    /* Case PDPOL=0*/                 
	   }
     p_Ion_WriteByte(CTL2,k);
     if(iic_active==1)
     {
     p_Ion_MultiWriteBytes(LDTH,&tmp_buffer[5],5);
     }
     else
     { 
     /* Set up LDTH */
     p_Ion_WriteByte(LDTH,tmp_buffer[5]);
  	 /* Set up PDTH */
  	 p_Ion_WriteByte(PDTH,tmp_buffer[6]);
  	 /* Set up PW */
  	 p_Ion_WriteByte(PW,tmp_buffer[7]);
  	 /* Set up LT */ 
  	 p_Ion_WriteByte(LT,tmp_buffer[8]);
  	 /* Set up TW */
  	 p_Ion_WriteByte(TW,tmp_buffer[9]);
     }
     ReadIonLDTH();
     ReadIonPDTH();
     ReadIonPW();
     ReadIonLT();
     ReadIonTW();
     
  	 Ion_SwitchMode(ION_PULSE_D);               
     mode = ION_DOUBLEPULSE_D;
     ReadIonMODE();
     ReadAllIonRegisters(); 
     Ion_ClearIntLatch();  // clear ION latches 
     break;
   
	   default:
	   break;
	}
  	
}

/**************************************************************
*	Function: 	Change_SensorInterface 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Configure communication IIC or SPI
**************************************************************/
void Change_SensorInterface(void)
{
  	if(iic_active)
  	{
  	  // sensor will communicate by IIC
  	  p_Ion_InitInterface = SwIIC_Init;
  	  p_Ion_ReadByte = SwIIC_ReadByte;
  	  p_Ion_WriteByte = SwIIC_WriteByte;
  	  p_Ion_MultiReadBytes = SwIIC_MultiReadBytes;
  	} else
  	{
  	  // sensor will communicate by SPI
  	  p_Ion_InitInterface = SwSPI_Init;
  	  p_Ion_ReadByte = SwSPI_ReadByte;
  	  p_Ion_WriteByte = SwSPI_WriteByte;
  	  p_Ion_MultiReadBytes = Dummy_Pointer;  	  
  	}	
}

/**************************************************************
*	Function: 	Ion_SwitchMode
*	Parameters: mode - new mode
*	Return:		none
*
*	Function switch working mode in ION
**************************************************************/

void Ion_SwitchMode(byte mode)
{
  byte i;
 	i = p_Ion_ReadByte(MCTL);
	i &= ~0x03;
	if(mode > 3) mode = 0;
	i |= mode;
	p_Ion_WriteByte(MCTL,i);
	ReadIonMCTL(); 
}

/**************************************************************
*	Function: 	Ion_ClearIntLatchINT1
*	Parameters: none
*	Return:		none
*
*	Function reset interrupts flags
**************************************************************/

void Ion_ClearIntLatchINT1(void)
{
  word i = 0;
         
  p_Ion_WriteByte(INTRST,0x01); 

  while(INT1 && (i++ < MAX_IIC_TIMEOUT));      //Optional

  p_Ion_WriteByte(INTRST,0x00); 
}

/**************************************************************
*	Function: 	Ion_ClearIntLatchINT2
*	Parameters: none
*	Return:		none
*
*	Function reset interrupts flags
**************************************************************/
void Ion_ClearIntLatchINT2(void)
{
  word i = 0;
          
  p_Ion_WriteByte(INTRST,0x02);

  while(INT2 && (i++ < MAX_IIC_TIMEOUT));      //Optional

  p_Ion_WriteByte(INTRST,0x00);
  
}

/**************************************************************
*	Function: 	Ion_ClearIntLatch
*	Parameters: none
*	Return:		none
*
*	Function reset interrupts flags
**************************************************************/
void Ion_ClearIntLatch(void)
{
  word i = 0;
      
  p_Ion_WriteByte(INTRST,0x03);      /* Double Pulse Detection, Clear INT1 & INT2 */

  while(INT2 && (i++ < MAX_IIC_TIMEOUT));      //Optional

  p_Ion_WriteByte(INTRST,0x00);
}

/**************************************************************
*	Function: 	AutoCalibration_Ion 
*	Parameters: None  
*	Return:		  Calibration 	 
*
*	Note:       No data retention in Ion MMA7455L at power off
*             requiring to recalibrate or reload calibration 
*             information 	
**************************************************************/
byte AutoCalibration_Ion(void)
{
  signed int tmpcalib[3];
  signed int tmpaccel[3];
  signed char tmpbuff[3]; 
  byte cnt_runs =0;
  byte res = 0;
 	byte i,j;

  for(i=0;i<3;i++)
  { 
    tmpcalib[i] = 0;
  }
  
 	SetUp_GselectIon(ION_G2);

 	while((cnt_runs++ < MAX_CNT_RUNS) && (res == 0))
 	{
 	  for(i=0;i<3;i++) tmpaccel[i] = 0; // Clear 
 	  
 	  for(i=0;i<4;i++)
 	  {
 	    
 	    while(!(INT1)); // wait for valid new data
      tmpbuff[0] = p_Ion_ReadByte(XOUT8);
     
      while(!(INT1)); // wait for valid new data
      tmpbuff[1] = p_Ion_ReadByte(YOUT8);
      
      while(!(INT1)); // wait for valid new data
      tmpbuff[2] = p_Ion_ReadByte(ZOUT8);

 	    for(j=0;j<3;j++) tmpaccel[j] += tmpbuff[j]; 	     
 	  }
 	  
 	  for(i=0;i<3;i++) tmpaccel[i] /= 4;
 	  
 	  tmpaccel[2] -= 0x3F;   //Sensitivity Z axis Ion adjustment 
 	                         // 0x3F is 1g at 2g range
 	  for(i=0;i<3;i++)
 	  {
 	    if(tmpaccel[i] == 0) res++;      
 	  }
 	  
 	  if(res >= 3) 
 	  {
 	    res = 1;
 	    for(j=0;j<3;j++)
 	      tmpaccel[j] += 1;
 	  } 	    
 	  else res = 0;
 	      
 	  for(i=0;i<3;i++) tmpcalib[i] -= tmpaccel[i];
 	  
 	  for(i=0;i<3;i++)
    {    	    
 	    p_Ion_WriteByte(XOFFL + i*2, (byte) (((word)tmpcalib[i]) % 256));
 	    p_Ion_WriteByte(XOFFH + i*2, (byte) (((word)tmpcalib[i]) / 256)); 	  
 	    calib[i] = tmpcalib[i];
 	  } 
 	  	  	  	      
 	}

	FlashErase1((byte*)calib_eeprom);  // Erase block of 512 Bytes - EEPROM segment
	CopyRam2FlashBurst((byte*)calib_eeprom, (byte*)tmpcalib, 6);   // destination adress, source address, length is 6 (important)

	return(res); 
}

/**************************************************************
*	Function: 	ReadAllIonRegisters 
*	Parameters: None   
*	Return:		 
*
*	Note:       Update Ion MMA7455L registers in 
*             Real-Time Debugger
**************************************************************/
void ReadAllIonRegisters(void) 
{
byte i;

if(iic_active) 
{ 
 p_Ion_MultiReadBytes(0,&ion_reg[0],32);  /* Read of all ION registers */
} 
else 
{
 for(i=0;i<32;i++)
 {  
 ion_reg[i]=p_Ion_ReadByte(i);
 }
}

ReadRegXOUTL.MergedBits.Reg=ion_reg[0]; 
ReadRegXOUTH.MergedBits.Reg=ion_reg[1]; 
ReadRegYOUTL.MergedBits.Reg=ion_reg[2]; 
ReadRegYOUTH.MergedBits.Reg=ion_reg[3]; 
ReadRegZOUTL.MergedBits.Reg=ion_reg[4];  
ReadRegZOUTH.MergedBits.Reg=ion_reg[5];  
ReadRegXOUT8.MergedBits.Reg=ion_reg[6]; 
ReadRegYOUT8.MergedBits.Reg=ion_reg[7]; 
ReadRegZOUT8.MergedBits.Reg=ion_reg[8]; 
ReadRegSTATUS.MergedBits.Reg=ion_reg[9]; 
ReadRegDETSRC.MergedBits.Reg=ion_reg[10]; 
ReadRegTOUT.MergedBits.Reg=ion_reg[11];
ReadRegRESERVED1.MergedBits.Reg=ion_reg[12]; 
ReadRegI2CAD.MergedBits.Reg=ion_reg[13]; 
ReadRegUSRINF.MergedBits.Reg=ion_reg[14]; 
ReadRegWHOAMI.MergedBits.Reg=ion_reg[15]; 
ReadRegXOFFL.MergedBits.Reg=ion_reg[16]; 
ReadRegXOFFH.MergedBits.Reg=ion_reg[17]; 
ReadRegYOFFL.MergedBits.Reg=ion_reg[18]; 
ReadRegYOFFH.MergedBits.Reg=ion_reg[19]; 
ReadRegZOFFL.MergedBits.Reg=ion_reg[20]; 
ReadRegZOFFH.MergedBits.Reg=ion_reg[21];
ReadRegMCTL.MergedBits.Reg=ion_reg[22];  
ReadRegINTRST.MergedBits.Reg=ion_reg[23]; 
ReadRegCTL1.MergedBits.Reg=ion_reg[24]; 
ReadRegCTL2.MergedBits.Reg=ion_reg[25]; 
ReadRegLDTH.MergedBits.Reg=ion_reg[26];  
ReadRegPDTH.MergedBits.Reg=ion_reg[27];  
ReadRegPW.MergedBits.Reg=ion_reg[28]; 
ReadRegLT.MergedBits.Reg=ion_reg[29];  
ReadRegTW.MergedBits.Reg=ion_reg[30];
ReadRegRESERVED2.MergedBits.Reg=ion_reg[31];
}

/**************************************************************
*	Function: initTPM1CH0 
*	Parameters:   
*	Return:		 
*
*	
**************************************************************/
void initTPM1CH0(unsigned int tPD,unsigned int tLT,unsigned int tTW)

{
/* Note: TPM1CH0 used through PTD0 */

/* Set up TPM1SC - TOF TOIE CPWMS CLKSB CLKSA PS2 PS1 PSO
	      Bit[7]    TOF  Timer Overflow Flag (FLAG)        
	                1 TPM counter has overflowed                                                    
                  0 TPM counter has not reached modulo or overflow                
        Bit[6]    TOIE  Timer Overflow Interrupt Enable 
                  1 TOF interrupts enabled                                                      
                  0 TOF interrupts inhibited (use software polling)           
        Bit[5]    CPWMS Center Aligned PWM Select                   
                  1 All TPMx channels operate in center aligned PWM mode                                                      
                  0 All TPMx channels operate as input capture, output compare, or edge aligned PWM mode                                        
        Bit[4:3]  CLKS Clock Source Select 
                  00 No clock selected (TPM disabled)                                                       
                  01 Bus Rate Clock (BUSCLK)
                  10 Fixed system clock (XCLK)
                  11 External source                                                                  
        Bit[2:1:0] PS Prescale Divisor Select  
                  000   1
                  001   2
                  010   4
                  011   8
                  100  16
                  101  32
                  110  64
                  111 128  */  
TPM1SC=0x0F;/* Bus Rate ClockBus & prescale of 128: CLOCK TIMER = FBUS/128 */
            /* Clock timer= 223kHz (4.5us) with 28 544 700Hz (35ns) Bus Clock  */
               
/* Timer Counter Registers TPM1CNTH:TPM1CNTL */ 
TPM1CNTH=0x00; /* 0xFFFF:  65535 */              
TPM1CNTL=0x00; /* 0xFF:    255   */

/* Timer Counter Modulo Registers TPM1MODL:TPM1MODH */
TPM1MODH=0x00;   /*             65535=294ms approximatively   */ 
TPM1MODL=0x00;   /* Disabled:   1LSB: 4.5us                   */


/* Timer Channel Status and Control Register TPMxCnSC
          TPMxCnSC - CHnF CHnIE MSnB:MSnA ELSnB:ELSnA 0 0 
        Bit[7]    CHnF Channel n Flag (FLAG)         
	                1 Input capture or output compare event occured                                                   
                  0 No input capture or output compare event occured                   
        Bit[6]    CHnIE 
                  1 Channel n interrupt requests enabled                                                     
                  0 Channel n interrupt requests disabled (use software polling)           
        Bit[5:4]  MSnB:MSnA Mode select for TPM Channel n                                                                           
        Bit[3:2]  ELSnB:ELSnA Edge/Level Bits                                                                 
        Bit[1:0]  00  
        
        CPWMS MSnB:MSnA   ELSnB:ELSnA   Mode            Configuration
        -----------------------------------------------------------------------------           
        X     XX          00            Pin not used for TPM channel; use as 
                                        external clock for the TPM or revert 
                                        to I/O
        -----------------------------------------------------------------------------
        0     00          01            Input Capture       Rising Edge  
                          10            Input Capture       Falling Edge  
                          11            Input Capture       Rising or falling Edge 
        -----------------------------------------------------------------------------                         
        0     01          00            Output Compare      Software compare only        
                          01            Output Compare      Toggle output on compare
                          10            Output Compare      Clear output on compare
                          11            Output Compare      Set output on compare
        -----------------------------------------------------------------------------
        0     1X          10            Edge Aligned PWM    High-true pulses 
                                                            (clear output on compare) 
                          X1            Edge Aligned PWM    Low-true pulses
                                                            (set output on compare) 
        -----------------------------------------------------------------------------
        1     XX          10            Center Aligned PWM  High-true pulses
                                                            (clear output on compare) 
                          X1            Center Aligned PWM  Low-true pulses
                                                            (set output on compare) 
        ----------------------------------------------------------------------------- */
  TPM1C0SC=0x10;  /* 0000 0100 Output Compare & Software compare only */ 
  
/* Timer Channel Value Register TPMxCnVH:TPMxCnVL */
/* Set up at 350ms far above double pulse detection time of 232ms */
/* tmp_buffer[13] = 0x08; (4ms) + tmp_buffer[14] = 0xDA; (218ms) + tmp_buffer[15] = 0x0A; (10ms) = 232ms */
/* Clock timer: 223kHz => 4.5us, 0xFFFF=294ms */
SetUp_TPM_Value1(tPD,tLT,tTW);
}


void SetUp_TPM_Value1(unsigned int tPD,unsigned int tLT,unsigned int tTW)
{
counterVALUE1=(tPD/2+tLT+tTW);     /* tPD: 1LSB is 0.5ms while tLT and tTW have 1LSB for 1ms */ 
counterVALUE1=counterVALUE1*1000;   /* Convert ms in us */
counterVALUE1=counterVALUE1/4;      /* Divide by 4 (period for counter is 4.5us) to load TPM1C0VH:TPM1C0VH */
counterVALUE1+=1000;               /* Add 1000 counts which is 4.5ms to have an additional delay */ 

counterVALUE1H=(unsigned char)(counterVALUE1>>8);       
TPM1C0VH=counterVALUE1H;                       
counterVALUE1L=(unsigned char)(counterVALUE1&0x00FF);            
TPM1C0VL=counterVALUE1L;                       
}                                             

/**************************************************************
*	Function: 	Reset_Counter 
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/
void Reset_Counter1(void)
{ 
/* Reset counter to 0 */
TPM1CNTH=0x00; /* Reset counter to 0 */              
TPM1CNTL=0x00; /* Reset counter to 0 */
}

/**************************************************************
*	Function: 	Clear_TOF
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/
void Clear_TOF1(void)
{
unsigned char tmp;

/* Clear Timer Overflow Flag (TOF) */
tmp=TPM1SC;
if(tmp==0x8F) TPM1SC&=0x7F;
}

/**************************************************************
*	Function: 	Clear_CHOF
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/  
void Clear_CHOF1(void) 
{  
unsigned char tmp;
  
/* Clear Channel 0 Flag (CH0F) */
tmp=TPM1C0SC;
if(tmp==0x90) TPM1C0SC&=0x10;
}

/**************************************************************
*	Function: initTPM2CH0 
*	Parameters:   
*	Return:		 
*
*	
**************************************************************/
void initTPM2CH0(unsigned int tPD,unsigned int tLT,unsigned int tTW)

{
/* Note: TPM2CH0 used through PTD0 */

/* Set up TPM1SC - TOF TOIE CPWMS CLKSB CLKSA PS2 PS1 PSO
	      Bit[7]    TOF  Timer Overflow Flag (FLAG)        
	                1 TPM counter has overflowed                                                    
                  0 TPM counter has not reached modulo or overflow                
        Bit[6]    TOIE  Timer Overflow Interrupt Enable 
                  1 TOF interrupts enabled                                                      
                  0 TOF interrupts inhibited (use software polling)           
        Bit[5]    CPWMS Center Aligned PWM Select                   
                  1 All TPMx channels operate in center aligned PWM mode                                                      
                  0 All TPMx channels operate as input capture, output compare, or edge aligned PWM mode                                        
        Bit[4:3]  CLKS Clock Source Select 
                  00 No clock selected (TPM disabled)                                                       
                  01 Bus Rate Clock (BUSCLK)
                  10 Fixed system clock (XCLK)
                  11 External source                                                                  
        Bit[2:1:0] PS Prescale Divisor Select  
                  000   1
                  001   2
                  010   4
                  011   8
                  100  16
                  101  32
                  110  64
                  111 128  */  
TPM2SC=0x0F;/* Bus Rate ClockBus & prescale of 128: CLOCK TIMER = FBUS/128 */
            /* Clock timer= 223kHz (4.5us) with 28 544 700Hz (35ns) Bus Clock  */
               
/* Timer Counter Registers TPM1CNTH:TPM1CNTL */ 
TPM2CNTH=0x00; /* 0xFFFF:  65535 */              
TPM2CNTL=0x00; /* 0xFF:    255   */

/* Timer Counter Modulo Registers TPM1MODL:TPM1MODH */
TPM2MODH=0x00;   /*             65535=294ms approximatively   */ 
TPM2MODL=0x00;   /* Disabled:   1LSB: 4.5us                   */


/* Timer Channel Status and Control Register TPMxCnSC
          TPMxCnSC - CHnF CHnIE MSnB:MSnA ELSnB:ELSnA 0 0 
        Bit[7]    CHnF Channel n Flag (FLAG)         
	                1 Input capture or output compare event occured                                                   
                  0 No input capture or output compare event occured                   
        Bit[6]    CHnIE 
                  1 Channel n interrupt requests enabled                                                     
                  0 Channel n interrupt requests disabled (use software polling)           
        Bit[5:4]  MSnB:MSnA Mode select for TPM Channel n                                                                           
        Bit[3:2]  ELSnB:ELSnA Edge/Level Bits                                                                 
        Bit[1:0]  00  
        
        CPWMS MSnB:MSnA   ELSnB:ELSnA   Mode            Configuration
        -----------------------------------------------------------------------------           
        X     XX          00            Pin not used for TPM channel; use as 
                                        external clock for the TPM or revert 
                                        to I/O
        -----------------------------------------------------------------------------
        0     00          01            Input Capture       Rising Edge  
                          10            Input Capture       Falling Edge  
                          11            Input Capture       Rising or falling Edge 
        -----------------------------------------------------------------------------                         
        0     01          00            Output Compare      Software compare only        
                          01            Output Compare      Toggle output on compare
                          10            Output Compare      Clear output on compare
                          11            Output Compare      Set output on compare
        -----------------------------------------------------------------------------
        0     1X          10            Edge Aligned PWM    High-true pulses 
                                                            (clear output on compare) 
                          X1            Edge Aligned PWM    Low-true pulses
                                                            (set output on compare) 
        -----------------------------------------------------------------------------
        1     XX          10            Center Aligned PWM  High-true pulses
                                                            (clear output on compare) 
                          X1            Center Aligned PWM  Low-true pulses
                                                            (set output on compare) 
        ----------------------------------------------------------------------------- */
  TPM2C0SC=0x10;  /* 0000 0100 Output Compare & Software compare only */ 
  
/* Timer Channel Value Register TPMxCnVH:TPMxCnVL */
/* Set up at 350ms far above double pulse detection time of 232ms */
/* tmp_buffer[13] = 0x08; (4ms) + tmp_buffer[14] = 0xDA; (218ms) + tmp_buffer[15] = 0x0A; (10ms) = 232ms */
/* Clock timer: 223kHz => 4.5us, 0xFFFF=294ms */
SetUp_TPM_Value2(tPD,tLT,tTW);
}  

void SetUp_TPM_Value2(unsigned int tPD,unsigned int tLT,unsigned int tTW)
{
counterVALUE2=(tPD/2+tLT+tTW);     /* tPD: 1LSB is 0.5ms while tLT and tTW have 1LSB for 1ms */ 
counterVALUE2=counterVALUE2*1000;   /* Convert ms in us */
counterVALUE2=counterVALUE2/4;      /* Divide by 4 (period for counter is 4.5us) to load TPM1C0VH:TPM1C0VH */
counterVALUE2+=1000;               /* Add 1000 counts which is 4.5ms to have an additional delay */ 

counterVALUE2H=(unsigned char)(counterVALUE2>>8);       
TPM2C0VH=counterVALUE2H;                       
counterVALUE2L=(unsigned char)(counterVALUE2&0x00FF);            
TPM2C0VL=counterVALUE2L;                       
}                                                 

/**************************************************************
*	Function: 	Reset_Counter 
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/
void Reset_Counter2(void)
{ 
/* Reset counter to 0 */
TPM2CNTH=0x00; /* Reset counter to 0 */              
TPM2CNTL=0x00; /* Reset counter to 0 */
}

/**************************************************************
*	Function: 	Clear_TOF
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/
void Clear_TOF2(void)
{
unsigned char tmp;

/* Clear Timer Overflow Flag (TOF) */
tmp=TPM2SC;
if(tmp==0x8F) TPM2SC&=0x7F;
}

/**************************************************************
*	Function: 	Clear_CHOF
*	Parameters: None  
*	Return:		 
*
*	
**************************************************************/  
void Clear_CHOF2(void) 
{  
unsigned char tmp;
  
/* Clear Channel 0 Flag (CH0F) */
tmp=TPM2C0SC;
if(tmp==0x90) TPM2C0SC&=0x10;
}
          
/**************************************************************
*	Function: 	Set_SelfTest 
*	Parameters: None  
*	Return:		 
*
*	Note:       Self test enabled will deflect Z axis by +1g
**************************************************************/
void Set_SelfTest(void)
{
	byte i;
	
	i = p_Ion_ReadByte(MCTL);
	i |= 0b00010000;
	p_Ion_WriteByte(MCTL,i);
	ReadIonMCTL();	
}

/**************************************************************
*	Function: 	Clear_SelfTest 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Disable self test
**************************************************************/
void Clear_SelfTest(void)
{
	byte i;
	
	i = p_Ion_ReadByte(MCTL);
	i &= 0b11101111;
	p_Ion_WriteByte(MCTL,i);
	ReadIonMCTL();	
}

/**************************************************************
*	Function: 	ReadIonXOUTL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update XOUTL in Real-Time Debugger
**************************************************************/
void ReadIonMODE(void)
{
if(mode==ION_STANDBY) ReadMODE.MergedBits.Reg=0x01; 
if(mode==ION_MEASURE) ReadMODE.MergedBits.Reg=0x02; 
if(mode==ION_LEVEL_D) ReadMODE.MergedBits.Reg=0x04; 
if(mode==ION_PULSE_D) ReadMODE.MergedBits.Reg=0x08; 
if(mode==ION_DOUBLEPULSE_D) ReadMODE.MergedBits.Reg=0x10; 
} 

/**************************************************************
*	Function: 	ReadIonXOUTL 
*	Parameters: None   
*	Return:		 
*
*	
**************************************************************/
void ReadIonXOUTL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(XOUTL); 
ReadRegXOUTL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonXOUTH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update XOUTH in Real-Time Debugger
**************************************************************/
void ReadIonXOUTH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(XOUTH); 
ReadRegXOUTH.MergedBits.Reg=tmp; 
}  

/**************************************************************
*	Function: 	ReadIonYOUTL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update YOUTL in Real-Time Debugger
**************************************************************/
void ReadIonYOUTL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(YOUTL); 
ReadRegYOUTL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonYOUTH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update YOUTH in Real-Time Debugger
**************************************************************/
void ReadIonYOUTH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(YOUTH); 
ReadRegYOUTH.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonZOUTL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update ZOUTL in Real-Time Debugger
**************************************************************/
void ReadIonZOUTL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(ZOUTL); 
ReadRegZOUTL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonZOUTH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update ZOUTH in Real-Time Debugger
**************************************************************/
void ReadIonZOUTH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(ZOUTH); 
ReadRegZOUTH.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonXOUT8 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update XOUT8 in Real-Time Debugger
**************************************************************/
void ReadIonXOUT8(void)
{
byte tmp;
tmp= p_Ion_ReadByte(XOUT8); 
ReadRegXOUT8.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonYOUT8 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update YOUT8 in Real-Time Debugger
**************************************************************/
void ReadIonYOUT8(void)
{
byte tmp;
tmp= p_Ion_ReadByte(YOUT8); 
ReadRegYOUT8.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonZOUT8 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update ZOUT8 in Real-Time Debugger
**************************************************************/
void ReadIonZOUT8(void)
{
byte tmp;
tmp= p_Ion_ReadByte(ZOUT8); 
ReadRegZOUT8.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonSTATUS 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update STATUS in Real-Time Debugger
**************************************************************/
void ReadIonSTATUS(void)
{
byte tmp;
tmp= p_Ion_ReadByte(STATUS); 
ReadRegSTATUS.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonDETSR 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update DETSR in Real-Time Debugger
**************************************************************/ 
void ReadIonDETSR(void)
{
byte tmp;
tmp= p_Ion_ReadByte(DETSRC); 
ReadRegDETSRC.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonTOUT 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update TOUT in Real-Time Debugger
**************************************************************/
void ReadIonTOUT(void) 
{
byte tmp;
tmp= p_Ion_ReadByte(TOUT); 
ReadRegTOUT.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonI2CAD 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update I2CAD in Real-Time Debugger
**************************************************************/
void ReadIonI2CAD(void)
{
byte tmp;
tmp= p_Ion_ReadByte(I2CAD); 
ReadRegI2CAD.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonUSRINF 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update USRINF in Real-Time Debugger
**************************************************************/
void ReadIonUSRINF(void)
{
byte tmp;
tmp= p_Ion_ReadByte(USRINF); 
ReadRegUSRINF.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonWHOAMI 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update WHOAMI in Real-Time Debugger
**************************************************************/
void ReadIonWHOAMI(void)
{
byte tmp;
tmp= p_Ion_ReadByte(WHOAMI); 
ReadRegWHOAMI.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonXOFFL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update XOFFL in Real-Time Debugger
**************************************************************/
void ReadIonXOFFL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(XOFFL); 
ReadRegXOFFL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonXOFFH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update XOFFH in Real-Time Debugger
**************************************************************/
void ReadIonXOFFH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(XOFFH); 
ReadRegXOFFH.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonYOFFL  
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update YOFFL in Real-Time Debugger
**************************************************************/
void ReadIonYOFFL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(YOFFL); 
ReadRegYOFFL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonYOFFH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update YOFFH in Real-Time Debugger
**************************************************************/
void ReadIonYOFFH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(YOFFH); 
ReadRegYOFFH.MergedBits.Reg=tmp; 
}  

/**************************************************************
*	Function: 	ReadIonZOFFL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update ZOFFL in Real-Time Debugger
**************************************************************/
void ReadIonZOFFL(void)
{
byte tmp;
tmp= p_Ion_ReadByte(ZOFFL); 
ReadRegZOFFL.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonZOFFH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update ZOFFH in Real-Time Debugger
**************************************************************/
void ReadIonZOFFH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(ZOFFH); 
ReadRegZOFFH.MergedBits.Reg=tmp; 
}  

/**************************************************************
*	Function: 	ReadIonINTRST 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update INTRST in Real-Time Debugger
**************************************************************/
void ReadIonINTRST(void)
{
byte tmp;
tmp= p_Ion_ReadByte(INTRST); 
ReadRegINTRST.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonMCTL 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update MCTL in Real-Time Debugger
**************************************************************/
void ReadIonMCTL(void)                  
{
byte tmp;
tmp= p_Ion_ReadByte(MCTL); 
ReadRegMCTL.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonCTL1 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update CTL1 in Real-Time Debugger
**************************************************************/
void ReadIonCTL1(void)                  
{
byte tmp;
tmp= p_Ion_ReadByte(CTL1); 
ReadRegCTL1.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonCTL2 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update CTL2 in Real-Time Debugger
**************************************************************/
void ReadIonCTL2(void)
{
byte tmp;
tmp= p_Ion_ReadByte(CTL2); 
ReadRegCTL2.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonLDTH 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update LDTH in Real-Time Debugger
**************************************************************/
void ReadIonLDTH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(LDTH); 
ReadRegLDTH.MergedBits.Reg=tmp; 
}

/**************************************************************
*	Function: 	ReadIonPDTH 
*	Parameters: None  
*	Return:		  None
*
*	Note:       Update PDTH in Real-Time Debugger
**************************************************************/
void ReadIonPDTH(void)
{
byte tmp;
tmp= p_Ion_ReadByte(PDTH); 
ReadRegPDTH.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonPW 
*	Parameters: None 
*	Return:		  None
*
*	Note:       Update PW in Real-Time Debugger
**************************************************************/
void ReadIonPW(void)
{
byte tmp;
tmp= p_Ion_ReadByte(PW); 
ReadRegPW.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonLT 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update LT in Real-Time Debugger
**************************************************************/
void ReadIonLT(void)
{
byte tmp;
tmp= p_Ion_ReadByte(LT); 
ReadRegLT.MergedBits.Reg=tmp; 
} 

/**************************************************************
*	Function: 	ReadIonTW 
*	Parameters: None   
*	Return:		  None
*
*	Note:       Update TW in Real-Time Debugger
**************************************************************/
void ReadIonTW(void) 
{
byte tmp;
tmp= p_Ion_ReadByte(TW); 
ReadRegTW.MergedBits.Reg=tmp; 
}    








