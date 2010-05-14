//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descrição: Firware do MSP430 utilizado no controle remmoto.
//#############################################################################

#include <msp430.h>

//#############################################################################
//#############################################################################

#define PIN_LED1    BIT4    // Porta 2
#define PIN_LED2    BIT2    // Porta 2
#define PIN_LED3    BIT3    // Porta 2

#define PIN_INT1    BIT0    // Porta 2
#define PIN_INT2    BIT1    // Porta 2

#define PIN_MOSI    BIT1    // Porta 3
#define PIN_MISO    BIT2    // Porta 3
#define PIN_SCK     BIT3    // Porta 3
#define PIN_CS      BIT4    // Porta 4

//#############################################################################
//#############################################################################

int main( void )
{
  WDTCTL = WDTPW | WDTHOLD;                  // Interrompe o watchdog timer.
  
  BCSCTL1 = CALBC1_16MHZ;                    // Configura o DCO para 16MHz.
  DCOCTL  = CALDCO_16MHZ;
  
  BCSCTL2 = BCSCTL2 | DIVS_3;                // Configura SMCLK para 2MHz.

  P2DIR = P2DIR & (~PIN_INT1) & (~PIN_INT2); // Configura pinos INT1 e INT2.
  P2IE  = P2IE & PIN_INT1 & PIN_INT2;
  P2IES = 0;
  P2IFG = P2IFG & (~PIN_INT1) & (~PIN_INT2);

  P4DIR = P4DIR | PIN_CS;                         // Configura pino CS.
  P4OUT = P4DIR | PIN_CS;
  
  P3SEL = P3SEL | PIN_MOSI | PIN_MISO | PIN_SCK;  // Configura pinos SPI.

}

//#############################################################################
//#############################################################################

#pragma vector = PORT_INT_VECTOR
__interrupt void interruption_vector(void)
{

}

//#############################################################################
//#############################################################################