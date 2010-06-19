#include "include.h"

char txBuffer[4];
char rxBuffer[4];
unsigned int i = 0;

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

  TI_CC_SPIStrobe(SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
}

// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
  // If Switch was pressed
  if(P1IFG & PIN_SWITCH)
  {
    // Build packet
    txBuffer[0] = 2;                        // Packet length
    txBuffer[1] = 0x01;                     // Packet address
    txBuffer[2] = (~P1IFG << 1) & 0x02; // Load switch inputs
    RFSendPacket(txBuffer, 3);              // Send value over RF
    __delay_cycles(5000);                   // Switch debounce
  }
  P1IFG &= ~(PIN_SWITCH);           // Clr flag that caused int
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
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