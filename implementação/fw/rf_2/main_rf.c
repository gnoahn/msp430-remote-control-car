#include "include.h"

char txBuffer[4];
char rxBuffer[4];

void main (void)
{
  WDTCTL = WDTPW | WDTHOLD;

  SPIInitialization();
  RFInitialization();
  RFConfiguration();

  // Configure Switch
  P1REN = PIN_SWITCH;
  P1IES = PIN_SWITCH;
  P1IFG = P1IFG & (~PIN_SWITCH);
  P1IE  = PIN_SWITCH;
  
  // Configure LEDs
  P1OUT = P1OUT & (~PIN_LED1) & (~PIN_LED2);
  P1DIR = P1DIR | PIN_LED1 | PIN_LED2;

  // Configure GDO0 to RX packet info from CC2500
  P2IES = P2IES | PIN_GDO0;
  P2IFG = P2IFG & (~PIN_GDO0);
  P2IE  = P2IE | PIN_GDO0;

  WriteStrobe(SRX);

  __bis_SR_register(LPM3_bits + GIE);
}

#pragma vector=PORT1_VECTOR // Interrupt from a pressed button
__interrupt void Port1_ISR (void)
{
    txBuffer[0] = 2;
    txBuffer[1] = 0x01;
    txBuffer[2] = (~P1IFG << 1) & 0x02;
    
    RFSend(txBuffer, 3);
    
    __delay_cycles(5000);
    
    P1IFG = P1IFG & (~PIN_SWITCH);
}

#pragma vector=PORT2_VECTOR // Interrupt from GDO0 (packet received)
__interrupt void Port2_ISR(void)
{
    // if GDO fired
  if(P2IFG & PIN_GDO0)
  {
    char len=2;                             // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer,&len))     // Fetch packet from CCxxxx
    P1OUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data
  }

  P2IFG &= ~PIN_GDO0;      // After pkt RX, this flag is set.
}