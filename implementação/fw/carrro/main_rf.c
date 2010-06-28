#include "include.h"

char rxBuffer[4];

void main (void)
{
  WDTCTL = WDTPW | WDTHOLD;

  SPIInitialization();
  RFInitialization();
  RFConfiguration();

  // Configure LEDs
//  P1OUT = P1OUT & (~PIN_LED1) & (~PIN_LED2);
//  P1DIR = P1DIR | PIN_LED1 | PIN_LED2;
  
  P2OUT = P2OUT & (~LED1) & (~LED2) & (~LED3);
  P2DIR = P2DIR | LED1 | LED2 | LED3;

  // Configure GDO0 to RX packet info from CC2500
  P2IES = P2IES | PIN_GDO0;
  P2IFG = P2IFG & (~PIN_GDO0);
  P2IE  = P2IE | PIN_GDO0;

  WriteStrobe(SRX);

  __bis_SR_register(LPM3_bits + GIE);
}

#pragma vector = PORT2_VECTOR // Interrupt from GDO0 (packet received)
__interrupt void port2_ISR(void)
{
  if (RFReceivePacket(rxBuffer, 2))
  {
    switch (rxBuffer[1])
    {
    case 0x11: P2OUT = P2OUT | LED1; // Turn on LED1
               break;
    case 0x12: P2OUT = P2OUT | LED2; // Turn on LED2
               break;
    case 0x13: P2OUT = P2OUT | LED3; // Turn on LED3
               break;
    case 0x01: P2OUT = P2OUT & (~LED1); // Turn off LED1
               break;
    case 0x02: P2OUT = P2OUT & (~LED2); // Turn off LED2
               break;
    case 0x03: P2OUT = P2OUT & (~LED3); // Turn off LED3
               break;
    case 0x04: P2OUT = P2OUT & (~LED1) & (~LED3); // Turn off LED1 and LED3
               break;
    case 0x05: P2OUT = P2OUT & (~LED1) & (~LED2) & (~LED3); // Turn off LED1 and LED3
               break;
    }
  }
  P2IFG = P2IFG & (~PIN_GDO0);
}
