//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descrição: Firware do MSP430 utilizado no controle remoto.
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

#define XOUTL       0x00    // LSB de X em 10 bits
#define XOUTH       0x01    // MSB de X em 10 bits
#define YOUTL       0x02    // LSB de Y em 10 bits
#define YOUTH       0x03    // MSB de Y em 10 bits
#define ZOUTL       0x04    // LSB de Z em 10 bits
#define ZOUTH       0x05    // MSB de Z em 10 bits
#define XOUT8       0x06    // X em 8 bits
#define YOUT8       0x07    // Y em 8 bits
#define ZOUT8       0x08    // Z em 8 bits
#define STATUS      0x09    // Registrador de STATUS
#define DETSRC      0x0A    // Fonte de detecção
#define TOUT        0x0B    // Temperatura
#define I2CAD       0x0D    // Endereço do dispositivo I2C
#define USRINF      0x0E    // Informação do usuário
#define WHOAMI      0x0F    // Identificação do dispositivo
#define XOFFL       0x10    // LSB de calibração em X
#define XOFFH       0x11    // MSB de calibração em X
#define YOFFL       0x12    // LSB de calibração em Y
#define YOFFH       0x13    // MSB de calibração em Y
#define ZOFFL       0x14    // LSB de calibração em Z
#define ZOFFH       0x15    // MSB de calibração em Z
#define MCTL        0x16    // Modo de funcionamento
#define INTRST      0x17    // Interrupções
#define CTL1        0x18    // Registrador de controle 1
#define CTL2        0x19    // Registrador de controle 2
#define LDTH        0x1A    // Limite do nível de detecção
#define PDTH        0x1B    // Limite do pulso de detecção
#define PW          0x1C    // Duração do pulso
#define LT          0x1D    // Tempo de latência
#define TW          0x1E    // Janela de tempo para o 2o pulso

//#############################################################################
//#############################################################################

unsigned char ReadRegister(unsigned char address);
unsigned char WriteRegister(unsigned char address, unsigned char data);

//#############################################################################
//#############################################################################

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                  // Interrompe o watchdog timer.
  
  BCSCTL1 = CALBC1_16MHZ;                    // Configura o DCO para 16MHz.
  DCOCTL  = CALDCO_16MHZ;
  
  BCSCTL2 = BCSCTL2 | DIVS_0;                // Configura SMCLK para 16MHz.

  P2DIR = P2DIR & (~PIN_INT1) & (~PIN_INT2); // Configura pinos INT1 e INT2.
  P2IE  = P2IE & PIN_INT1 & PIN_INT2;
  P2IES = 0;
  P2IFG = P2IFG & (~PIN_INT1) & (~PIN_INT2);

  P4DIR = P4DIR | PIN_CS;                         // Configura pino CS.
  P4OUT = P4OUT | PIN_CS;
  
  P3SEL = P3SEL | PIN_MOSI | PIN_MISO | PIN_SCK;  // Configura pinos SPI.

  UCB0CTL0 = UCB0CTL0 | UCSYNC | UCMODE_1 | UCMST | UCMSB | UCCKPH;    // Configura SPI.
  UCB0CTL1 = UCB0CTL1 | UCSSEL_2;
  UCB0BR0  = 0x02;
  UCB0BR1  = 0x00;
  UCB0CTL1 = UCB0CTL1 & (~UCSWRST);
}

//#############################################################################
//#############################################################################

#pragma vector = PORT_INT_VECTOR
__interrupt void interruption_vector(void)
{
  if (P2IN & PIN_INT1)
  {
    xdata = ReadRegister(XOUT8);
    delay(??);
    ydata = ReadRegister(YOUT8);
    delay(??);
    zdata = ReadRegister(ZOUT8);
    P2IFG = P2IFG & (~PIN_INT1);
  }
}

//#############################################################################
//#############################################################################

unsigned char ReadRegister(unsigned char address)
{
  unsigned char data;
  unsigned char trash;
  
  P4OUT = P4OUT & (~PIN_CS);  
  
  trash = UCB0RXBUF;
  
  UCB0TXBUF = address;
  while (!IFG2_bit.UCB0RXIFG);
  trash = UCB0RXBUF;
  
  UCB0TXBUF = 0;                  // trash = 0; UCB0TXBUF = trash;
  while (!IFG2_bit.UCB0RXIFG);
  data = UCB0RXBUF;
  
  P4OUT = P4OUT | PIN_CS;
  
  return data;
}

//#############################################################################
//#############################################################################

unsigned char WriteRegister(unsigned char address, unsigned char data)
{
  unsigned char trash;
  
  P4OUT = P4OUT & (~PIN_CS);
  
  trash = UCB0RXBUF;
  
  UCB0TXBUF = address;
  while (!IFG2_bit.UCB0RXIFG);
  trash = UCB0RXBUF;
  
  UCB0TXBUF = data;
  while (!IFG2_bit.UCB0RXIFG);
  trash = UCB0RXBUF;
  
  P4OUT = P4OUT | PIN_CS;
  
  return(0);
}

//#############################################################################
//#############################################################################