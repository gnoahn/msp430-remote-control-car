//#############################################################################
// Projeto: MSP430 Remote Control Car
// Descri��o: Os LEDs acendem de acordo com a inclina��o do aceler�metro.
//#############################################################################

#include "msp430.h"
#include "include.h" 

char txBuffer[4];
char rxBuffer[4];

//#############################################################################
//#############################################################################

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
#define DETSRC      0x0A    // Fonte de detec��o
#define TOUT        0x0B    // Temperatura
#define I2CAD       0x0D    // Endere�o do dispositivo I2C
#define USRINF      0x0E    // Informa��o do usu�rio
#define WHOAMI      0x0F    // Identifica��o do dispositivo
#define XOFFL       0x10    // LSB de calibra��o em X
#define XOFFH       0x11    // MSB de calibra��o em X
#define YOFFL       0x12    // LSB de calibra��o em Y
#define YOFFH       0x13    // MSB de calibra��o em Y
#define ZOFFL       0x14    // LSB de calibra��o em Z
#define ZOFFH       0x15    // MSB de calibra��o em Z
#define MCTL        0x16    // Modo de funcionamento
#define INTRST      0x17    // Interrup��es
#define CTL1        0x18    // Registrador de controle 1
#define CTL2        0x19    // Registrador de controle 2
#define LDTH        0x1A    // Limite do n�vel de detec��o
#define PDTH        0x1B    // Limite do pulso de detec��o
#define PW          0x1C    // Dura��o do pulso
#define LT          0x1D    // Tempo de lat�ncia
#define TW          0x1E    // Janela de tempo para o 2o pulso

//#############################################################################
//#############################################################################

unsigned char xdata;
unsigned char ydata;
unsigned char zdata;

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

  SPIInitialization();
  RFInitialization();
  RFConfiguration();

  WriteRegister(MCTL, 0x01);      // 8g = 0x01
  
  if (ReadRegister(MCTL) == 0x01)
  {
    P2OUT = P2OUT | PIN_LED2;
  }

  if (RFReadRegister(TEST1) == 0x31)
  {
    P2OUT = P2OUT | PIN_LED1;
    
    txBuffer[0] = 2;
    txBuffer[1] = 0x01;
    txBuffer[2] = (~P1IFG << 1) & 0x02;
    BurstWriteRegister(TXFIFO, txBuffer, 3);
    WriteStrobe(STX);
    __delay_cycles(5000);
  }
}