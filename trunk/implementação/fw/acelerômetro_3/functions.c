//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descrição: Funções diversas relacionadas a comunicação SPI.
//#############################################################################

#include "include.h"

//#############################################################################
//#############################################################################

unsigned char ReadRegister(unsigned char address)
{
  unsigned char data;
  unsigned char trash;
  
  P4OUT = P4OUT & (~PIN_CS_ACC);  
  
  trash = UCB0RXBUF;
  
  UCB0TXBUF = address << 1; // Read command -> MSB = 0, LSB = X
  while (!(IFG2 & UCB0RXIFG));
  trash = UCB0RXBUF;
  
  UCB0TXBUF = 0;            // trash = 0; UCB0TXBUF = trash;
  while (!(IFG2 & UCB0RXIFG));
  data = UCB0RXBUF;
  
  P4OUT = P4OUT | PIN_CS_ACC;
  
  return data;
}

//#############################################################################
//#############################################################################

unsigned char WriteRegister(unsigned char address, unsigned char data)
{
  unsigned char trash;
  
  P4OUT = P4OUT & (~PIN_CS_ACC);
  
  trash = UCB0RXBUF;
  
  address = (address << 1) | 0x80; // Write command -> MSB = 1
  
  UCB0TXBUF = address;
  while (!(IFG2 & UCB0RXIFG));
  trash = UCB0RXBUF;
  
  UCB0TXBUF = data;
  while (!(IFG2 & UCB0RXIFG));
  trash = UCB0RXBUF;
  
  P4OUT = P4OUT | PIN_CS_ACC;
  
  return(0);
}

//#############################################################################
//#############################################################################