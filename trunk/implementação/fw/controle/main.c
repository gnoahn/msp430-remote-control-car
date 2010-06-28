#include "msp430.h"
#include "include.h"

unsigned char xdata;
unsigned char ydata;
unsigned char zdata;

//#############################################################################
//#############################################################################

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;
  
  BCSCTL1 = CALBC1_16MHZ;
  DCOCTL  = CALDCO_16MHZ;
  
  BCSCTL2 = BCSCTL2 | DIVS_0;

  P2OUT = P2OUT & (~PIN_LED1) & (~PIN_LED2) & (~PIN_LED3);
  P2DIR = P2DIR | PIN_LED1 | PIN_LED2 | PIN_LED3;

  SPIInitialization();
  RFInitialization();
  RFConfiguration();

  WriteRegister(MCTL, 0x01);
  
  for(;;)
  {
    xdata = ReadRegister(XOUT8);
    ydata = ReadRegister(YOUT8);
    zdata = ReadRegister(ZOUT8);
    
    if (((zdata < 0x1F) & (zdata > 0x0B)) & ((xdata > 0x0B) & (xdata < 0x1F)))  // se p/ cima e inclinado p/ frente, então vai p/ FRENTE
    {
      P2OUT = P2OUT | PIN_LED2; RFSendData(0x12);
      if ((ydata < 0xFC) & (ydata > 0xE1)) {P2OUT = P2OUT | PIN_LED3; RFSendData(0x13);} // DIREITA  (y): -2g < G < 0g
      if ((ydata > 0x04) & (ydata < 0x1F)) {P2OUT = P2OUT | PIN_LED1; RFSendData(0x11);} // ESQUERDA (y): 0g < G < +2g
      if ((ydata > 0xFE) | (ydata < 0x02)) {P2OUT = P2OUT & (~PIN_LED1) & (~PIN_LED3); RFSendData(0x04);} // REPOUSO  (y): ~0g     
    }
    
    if (((zdata < 0x1F) & (zdata > 0x0B)) & ((xdata < 0x06) | (xdata > 0xFC))) // se p/ cima e não inclinado p/ frente, então não vai p/ FRENTE
    {
      P2OUT = P2OUT & (~PIN_LED2); RFSendData(0x02);
      if ((ydata < 0xFC) & (ydata > 0xE1)) {P2OUT = P2OUT | PIN_LED3; RFSendData(0x13);} // DIREITA  (y): -2g < G < 0g
      if ((ydata > 0x04) & (ydata < 0x1F)) {P2OUT = P2OUT | PIN_LED1; RFSendData(0x11);} // ESQUERDA (y): 0g < G < +2g
      if ((ydata > 0xFE) | (ydata < 0x02)) {P2OUT = P2OUT & (~PIN_LED1) & (~PIN_LED3); RFSendData(0x04);} // REPOUSO  (y): ~0g  
    }
    
    if ((zdata > 0x80) | (zdata < 0x06)) // se p/ baixo, então não faz NADA
    {
      P2OUT = P2OUT & (~PIN_LED1) & (~PIN_LED2) & (~PIN_LED3); RFSendData(0x05);
    }
  }
}