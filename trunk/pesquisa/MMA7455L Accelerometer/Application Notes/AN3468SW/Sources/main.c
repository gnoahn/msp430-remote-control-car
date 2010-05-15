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

#include "derivative.h" /* include peripheral declarations */
#include "doonstack.h"
#include "main.h"
#include "my_types.h"
#include "mcu_init.h"
#include "sw_spi.h"
#include "sw_iic.h"
#include "sci.h"


#pragma DATA_SEG SHORT _DATA_ZEROPAGE

byte n; 
byte X,Y,Z;                              //Ion MMA7455L acceleration data (8Bits)
byte ion_reg[32];                        //Array to read content of Ion MMA7455L registers                     
signed int calib[3];

byte mode = ION_MEASURE;

byte iic_active = 1;  //IIC (select "1") or SPI (select "0")  

void (*p_Ion_InitInterface)(void) = SwIIC_Init;
byte (*p_Ion_ReadByte)(byte) = SwIIC_ReadByte;
void (*p_Ion_WriteByte)(byte, byte) = SwIIC_WriteByte;
void (*p_Ion_MultiReadBytes)(byte, byte*, byte) = SwIIC_MultiReadBytes;

#pragma DATA_SEG DEFAULT

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
*	Return:		  None
*
*	Note:       Dummy function 
*             (SPI does not allow multiple write/read)
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
      LED1_OFF;
      Delay_Xms(200);
      LED1_ON;
      
      Set_SelfTest();        //Specific routine
             
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
      LED2_OFF;
      Delay_Xms(200);
      LED2_ON;
           
      Clear_SelfTest();
      ReadAllIonRegisters(); //Specific routine
 	       
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
      LED3_OFF;
      Delay_Xms(200);
      LED3_ON; 

      SetUp_ModeOfSensorIon(ION_G8);   //Specific routine
       
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
      LED4_OFF;
      Delay_Xms(200);
      LED4_ON;
      
      SetUp_ModeOfSensorIon(ION_G2);   //Specific routine
  
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
    MCU_Init();
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
    // Init Ion (measure mode 2g range selected)  	
  	Init_SensorIon();
  	Delay_Xms(50);
    LED3_ON; 
    
    // Initialization flash memory (store calibration data)
    FCDIV = FCDIV_PRDIV8_MASK | (10 & FCDIV_DIV_MASK); 
    
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
    
    //Read Ion Registers (view in Real-Time Debugger)
    ReadAllIonRegisters();
    
   for(;;)
   {      
      LED1_ON;
      LED2_ON;
      LED3_ON;
      LED4_ON;
      
      CheckButt0(); 
      CheckButt1();
      CheckButt2();
      CheckButt3();
          
  	  Read_AccelerometerIon(); 
  	
  	  SCI_Display();

      LED1_OFF;
      LED2_OFF;
      LED3_OFF;
      LED4_OFF;
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
      SCI_TxByte(X);
    
      /*Parameter2 */
      SCI_TxByte(0xFE); //FE01 identifies Y axis acceleration (8Bits)
      SCI_TxByte(0x01);
      SCI_TxByte(Y);
    
      /*Parameter3 */
      SCI_TxByte(0xFD); //FD02 identifies Z axis acceleration (8Bits)
      SCI_TxByte(0x02);
      SCI_TxByte(Z);
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
    for(k=0;k<7;k++)
    {
      for(j=0;j<200;j++)        
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
*	Parameters: None  
*	Return:		  None
*
* Note:       Selected: measure mode 2g 125Hz (62.5BW filter)	
**************************************************************/
void Init_SensorIon(void)
{
	p_Ion_InitInterface();         //Initialization of communication 
	INIT_INT_MACRO                 // IIC (iic_active=1)  
	Delay_Xms(20);                 // SPI (iic_active=0) 
	
	p_Ion_WriteByte(MCTL,0x05);    //DRDP=0  (data ready is output on to INT1/DRDY pin)
	ReadIonMCTL();                 //SPI3W=0 (SPI is 4 wire mode)
	                               //STON=0  (self test is not enabled)
	                               //GLVL[1:0]=01 (2g range) 
	                               //MODE[1:0]=01 (measurement mode)
	
	
	  p_Ion_WriteByte(CTL1,0x00);  //This is to set up DFBW: 62.5Hz for data rate 125Hz 
	//p_Ion_WriteByte(CTL1,0x80);  //This is to set up DFBW:  125Hz for data rate 250Hz                
  ReadIonCTL1();
}

/**************************************************************
*	Function: 	SetUp_GselectIon 
*	Parameters: gsel (ION_G2, ION_G4 or ION_G8) 
*	Return:		  None
*
*	Note:       Set up g range (2g, 4g or 8g)	
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
*	Parameters: gsel (ION_G2, ION_G4 or ION_G8)  
*	Return:		  None
*
*	Note:       Set up or change mode for acceleration sensor 	
**************************************************************/
void SetUp_ModeOfSensorIon(byte gsel)
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
	  

	  
	   Ion_SwitchMode(ION_MEASURE);                                            
     
     k = p_Ion_ReadByte(MCTL);
     k|=0x01;          //MODE[1:0]=01 (measurement mode)
	   k &= 0xB3;        //DRDP=0 (data ready is output on to INT1/DRDY pin) and GLVL[1:0]=00 (8g)
	   k |= (gsel << 2); //GLVL[1:0]=gsel  
	   p_Ion_WriteByte(MCTL,k);
	   ReadIonMCTL();
	   
	   p_Ion_WriteByte(CTL1,0x00);    //This is to set up DFBW: 62.5Hz for data rate 125Hz 
	   //SwSPI_WriteByte(CTL1,0x80);  //This is to set up DFBW:  125Hz for data rate 250Hz                                                
	   ReadIonCTL1();                 
     
  	 mode = ION_MEASURE;
  	 
  	 //Read Ion Registers (view in Real-Time Debugger)
     ReadAllIonRegisters();    	
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
*	Parameters: mode (measure)  
*	Return:		  None
*
*	Note:       Set up measure mode	
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



