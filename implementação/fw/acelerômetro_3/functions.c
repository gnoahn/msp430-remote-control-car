#include "include.h"

unsigned char WriteRegister(unsigned char address, unsigned char data)
{
  P4OUT = P4OUT & (~PIN_CS_ACC);
  
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = (address << 1) | 0x80;
  while (!(IFG2 & UCB0RXIFG));
  IFG2 = IFG2 & (~UCB0TXIFG) & (~UCB0RXIFG);
    
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = data;
  while (!(IFG2 & UCB0RXIFG));
  IFG2 = IFG2 & (~UCB0TXIFG) & (~UCB0RXIFG);
  
  P4OUT = P4OUT | PIN_CS_ACC;
  
  return (0);
}

unsigned char ReadRegister(unsigned char address)
{
  unsigned char data;
 
  P4OUT = P4OUT & (~PIN_CS_ACC);
  
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = address << 1;
  
  while (!(IFG2 & UCB0TXIFG));
  
  UCB0TXBUF = 0x00;
  
  while (!(IFG2 & UCB0RXIFG));
  IFG2 = IFG2 & (~UCB0RXIFG);
  
  while (!(IFG2 & UCB0RXIFG));
  
  data = UCB0RXBUF;
  
  P4OUT = P4OUT | PIN_CS_ACC;
  
  return data;
}