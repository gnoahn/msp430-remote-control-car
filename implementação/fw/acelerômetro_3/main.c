#include "include.h"

unsigned char x;

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

      UCB0CTL1 = UCB0CTL1 | UCSWRST;
  
      UCB0CTL0 = UCB0CTL0 | UCSYNC | UCMST | UCMSB | UCCKPH;    // Configura SPI.
      UCB0CTL1 = UCB0CTL1 | UCSSEL_2;
      UCB0BR0  = 0x08;                // SPI Speed = 16MHz/8 = 2MHz // Maximum Speed = 8MHz.
      UCB0BR1  = 0x00;
  
      P3SEL = P3SEL | PIN_SDATA | PIN_MISO | PIN_SCK; // Configura pinos SPI.
      P3DIR = P3DIR | PIN_SDATA | PIN_SCK;
  
      UCB0CTL1 = UCB0CTL1 & (~UCSWRST);

  WriteRegister(MCTL, 0x21);      // 8g = 0x01 // MCTL = 00100001
  x = ReadRegister (MCTL);
  
  if (x == 0x21) {P2OUT = P2OUT | PIN_LED1 | PIN_LED2 | PIN_LED3;}
}