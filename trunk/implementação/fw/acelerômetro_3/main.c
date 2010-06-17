//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descrição: Função principal.
//#############################################################################

#include "include.h"

//#############################################################################
//#############################################################################

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                      // Interrompe o watchdog timer.
  
  BCSCTL1 = CALBC1_16MHZ;                        // Configura o DCO para 16MHz.
  DCOCTL  = CALDCO_16MHZ;
  
  BCSCTL2 = BCSCTL2 | DIVS_0;                    // Configura SMCLK para 16MHz.

  P2OUT = P2OUT & ~PIN_LED1 & ~PIN_LED2 & ~PIN_LED3;
  P2DIR = P2DIR | PIN_LED1 | PIN_LED2 | PIN_LED3;

  P4OUT = P4OUT | PIN_CS_ACC;                    // Configura pino CS do acelerômetro.
  P4DIR = P4DIR | PIN_CS_ACC;

  P3SEL = P3SEL | PIN_MOSI | PIN_MISO | PIN_SCK; // Configura pinos SPI.

  UCB0CTL0 = UCB0CTL0 | UCSYNC | UCMODE_1 | UCMST | UCMSB | UCCKPH;    // Configura SPI.
  UCB0CTL1 = UCB0CTL1 | UCSSEL_2;
  UCB0BR0  = 0x08;                // SPI Speed = 16MHz/8 = 2MHz // Maximum Speed = 4MHz.
  UCB0BR1  = 0x00;
  UCB0CTL1 = UCB0CTL1 & (~UCSWRST);

  P4OUT = P4OUT & (~PIN_CS_ACC);  // Configura acelerômetro.
  WriteRegister(MCTL, 0x01);      // 8g = 0x01

  for(;;)
  {
    xdata = ReadRegister(XOUT8);
    ydata = ReadRegister(YOUT8);
    zdata = ReadRegister(ZOUT8);
    
    if (((zdata < 0x1F) & (zdata > 0x0B)) & ((xdata > 0x0B) & (xdata < 0x1F)))  // se p/ cima e inclinado p/ frente, então vai p/ FRENTE
    {
      P2OUT = P2OUT | PIN_LED2;
      if ((ydata < 0xFC) & (ydata > 0xE1)) {P2OUT = P2OUT | PIN_LED3;}              // DIREITA  (y): -2g < G < 0g
      if ((ydata > 0x04) & (ydata < 0x1F)) {P2OUT = P2OUT | PIN_LED1;}              // ESQUERDA (y): 0g < G < +2g
      if ((ydata > 0xFE) | (ydata < 0x02)) {P2OUT = P2OUT & ~PIN_LED1 & ~PIN_LED3;} // REPOUSO  (y): ~0g     
    }
    
    if (((zdata < 0x1F) & (zdata > 0x0B)) & ((xdata < 0x06) | (xdata > 0xFC))) // se p/ cima e não inclinado p/ frente, então não vai p/ FRENTE
    {
      P2OUT = P2OUT & ~PIN_LED2;
      if ((ydata < 0xFC) & (ydata > 0xE1)) {P2OUT = P2OUT | PIN_LED3;}              // DIREITA  (y): -2g < G < 0g
      if ((ydata > 0x04) & (ydata < 0x1F)) {P2OUT = P2OUT | PIN_LED1;}              // ESQUERDA (y): 0g < G < +2g
      if ((ydata > 0xFE) | (ydata < 0x02)) {P2OUT = P2OUT & ~PIN_LED1 & ~PIN_LED3;} // REPOUSO  (y): ~0g  
    }
    
    if ((zdata > 0x80) | (zdata < 0x06)) // se p/ baixo, então não faz NADA
    {
      P2OUT = P2OUT & ~PIN_LED1 & ~PIN_LED2 & ~PIN_LED3;
    }
  }
}