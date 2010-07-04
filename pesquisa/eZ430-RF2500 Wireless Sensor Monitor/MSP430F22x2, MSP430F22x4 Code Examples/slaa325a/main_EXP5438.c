//******************************************************************************
//  Demo Application for MSP430 Experimenters board F5438/CC1100-2500 Interface
//  Main code application library v1.1
//
// W. Goh
// Version 1.1
// Texas Instruments, Inc
// December 2009
// Built with IAR Embedded Workbench Version: 4.20
//******************************************************************************
// Change Log:
//******************************************************************************
// Version:  1.1
// Comments: Main application code designed for EXP5438 board
// Version:  1.00
// Comments: Initial Release Version
//******************************************************************************

#include "include.h"

extern char paTable[];
extern char paTableLen;

char txBuffer[4];
char rxBuffer[4];
unsigned int i = 0;

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  TI_CC_SPISetup();                         // Initialize SPI port

  TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
  writeRFSettings();                        // Write RF settings to config reg
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

  // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
  TI_CC_SW_PxREN |= TI_CC_SW1+TI_CC_SW2;    // Enable pull-up resistor
  TI_CC_SW_PxOUT |= TI_CC_SW1+TI_CC_SW2;    // Enable pull-up resistor
  TI_CC_SW_PxIES = TI_CC_SW1+TI_CC_SW2;     // Int on falling edge
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1+TI_CC_SW2); // Clr flags
  TI_CC_SW_PxIE = TI_CC_SW1+TI_CC_SW2;      // Activate interrupt enables
  TI_CC_LED_PxOUT &= ~(TI_CC_LED1 + TI_CC_LED2); // Outputs = 0
  TI_CC_LED_PxDIR |= TI_CC_LED1 + TI_CC_LED2;// LED Direction to Outputs

  TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
  TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

  TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
}


// The ISR assumes the interrupt came from a press of one of the two buttons
#pragma vector=PORT2_VECTOR
__interrupt void port2_ISR (void)
{
  // Build packet
  txBuffer[0] = 2;                          // Packet length
  txBuffer[1] = 0x01;                       // Packet address
  txBuffer[2] = (TI_CC_SW_PxIFG & 0xC0) >> 6; // Load switch inputs
  RFSendPacket(txBuffer, 3);                // Send value over RF
  __delay_cycles(100000);                   // Switch debounce

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear GDO flag
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1+TI_CC_SW2); // Clr flag that caused int
}                                           // Clear it.

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR (void)
{
  char len=2;                               // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
  switch(__even_in_range(P1IV,16))
  {
    case 0: break;
    case 2: break;                          // Vector P1IFG.0
    case 4: break;                          // Vector P1IFG.1
    case 6: break;                          // Vector P1IFG.2
    case 8: break;                          // Vector P1IFG.3
    case 10: break;                         // Vector P1IFG.4
    case 12: break;                         // Vector P1IFG.5
    case 14: break;                         // Vector P1IFG.6
    case 16:                                // Vector P1IFG.7
      if (RFReceivePacket(rxBuffer,&len))   // Fetch packet from CCxxxx
        TI_CC_LED_PxOUT ^= rxBuffer[1];     // Toggle LEDs according to pkt data
      break;
  }
}
