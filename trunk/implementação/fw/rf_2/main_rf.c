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

#pragma vector = PORT1_VECTOR // Interrupt from a pressed button
__interrupt void port1_ISR (void)
{
  unsigned int aux;
  txBuffer[0] = 2;
  txBuffer[1] = 0x01;
  txBuffer[2] = 0x02;
  
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = TXFIFO | WRITE_BURST_BIT;
  for (aux = 0; aux < 3; aux++)
  {
    while (!(IFG2 & UCB0TXIFG));
    UCB0TXBUF = txBuffer[aux];
  }
  while (UCB0STAT & UCBUSY);
  P3OUT = P3OUT | PIN_CS_RF;
  
  WriteStrobe(STX);
  __delay_cycles(5000);
  P1IFG = P1IFG & (~PIN_SWITCH);
}

#pragma vector = PORT2_VECTOR // Interrupt from GDO0 (packet received)
__interrupt void port2_ISR(void)
{
  if (RFReceivePacket(rxBuffer,2))
  {
    P1OUT = P1OUT ^ rxBuffer[1];
  }
  P2IFG = P2IFG & (~PIN_GDO0);
}