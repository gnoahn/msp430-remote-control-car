//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descrição: Firware do MSP430 utilizado no controle remmoto.
//#############################################################################

#include <msp430.h>

//#############################################################################
//#############################################################################

int main( void )
{
  WDTCTL = WDTPW | WDTHOLD;             // Interrompe o temporizador watchdog.
  
  BCSCTL1 = CALBC1_16MHZ;               // Configura o DCO para 16MHz.
  DCOCTL  = CALDCO_16MHZ;
  
  BCSCTL2 = BCSCTL2 | DIVS_3;           // Configura SMCLK para 2MHz.

  P4DIR = P4DIR & (~BIT3) & (~BIT5);   // Configura os pinos de interrupções.
  P4IE  = P4IE & BIT3 & BIT5;
  P4IES = 0;
  P4IFG = P4IFG & (~BIT3) & (~BIT5);

}

//#############################################################################
//#############################################################################

#pragma vector = PORT_INT_VECTOR
__interrupt void Port_INT_ISR(void)
{

}

//#############################################################################
//#############################################################################