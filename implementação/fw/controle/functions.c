#include "include.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned char RFSendData(unsigned char data)
{
  unsigned int aux;
  char txBuffer[4] = {2, 0x01, data};
  
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
  
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned char ReadRegister(unsigned char address)
{
  char data;
  P4OUT = P4OUT & (~PIN_CS_ACC);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = address << 1;
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = 0;
  while (UCB0STAT & UCBUSY);
  data = UCB0RXBUF;
  P4OUT = P4OUT | PIN_CS_ACC;
  return data;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned char WriteRegister(unsigned char address, unsigned char data)
{
  address = (address << 1) | 0x80;
  P4OUT = P4OUT & (~PIN_CS_ACC);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = address;
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = data;
  while (UCB0STAT & UCBUSY);
  P4OUT = P4OUT | PIN_CS_ACC;
  return(0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SPIInitialization(void)
{
  P3OUT = P3OUT | PIN_CS_RF;
  P3DIR = P3DIR | PIN_CS_RF;
  
  P4OUT = P4OUT | PIN_CS_ACC;
  P4DIR = P4DIR | PIN_CS_ACC;
  
  UCB0CTL1 = UCB0CTL1 | UCSWRST;
  UCB0CTL0 = UCB0CTL0 | UCMST | UCCKPH | UCMSB | UCSYNC;
  UCB0CTL1 = UCB0CTL1 | UCSSEL_2;
  UCB0BR0 = 0x02;
  UCB0BR1 = 0x00;
  
  P3SEL = P3SEL | PIN_MOSI | PIN_MISO | PIN_SCK;
  P3DIR = P3DIR | PIN_MOSI | PIN_SCK;

  UCB0CTL1 = UCB0CTL1 & (~UCSWRST);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void RFInitialization(void)
{
  P2SEL = 0; // Configure P2.6 (GDO0) and P2.7 (GDO2) as GPIOs
  
  P3OUT = P3OUT | PIN_CS_RF;
  __delay_cycles(30);
  P3OUT = P3OUT & (~PIN_CS_RF);
  __delay_cycles(30);
  P3OUT = P3OUT | PIN_CS_RF;
  __delay_cycles(45);
  P3OUT = P3OUT & (~PIN_CS_RF);
  
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = SRES;
  while (UCB0STAT & UCBUSY);
  
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void RFConfiguration(void)
{
    RFWriteRegister(IOCFG2,   0x0B); // GDO2 output pin config.
    RFWriteRegister(IOCFG0,   0x06); // GDO0 output pin config.
    RFWriteRegister(PKTLEN,   0xFF); // Packet length.
    RFWriteRegister(PKTCTRL1, 0x05); // Packet automation control.
    RFWriteRegister(PKTCTRL0, 0x05); // Packet automation control.
    RFWriteRegister(ADDR,     0x01); // Device address.
    RFWriteRegister(CHANNR,   0x00); // Channel number.
    RFWriteRegister(FSCTRL1,  0x07); // Freq synthesizer control.
    RFWriteRegister(FSCTRL0,  0x00); // Freq synthesizer control.
    RFWriteRegister(FREQ2,    0x5D); // Freq control word, high byte
    RFWriteRegister(FREQ1,    0x93); // Freq control word, mid byte.
    RFWriteRegister(FREQ0,    0xB1); // Freq control word, low byte.
    RFWriteRegister(MDMCFG4,  0x2D); // Modem configuration.
    RFWriteRegister(MDMCFG3,  0x3B); // Modem configuration.
    RFWriteRegister(MDMCFG2,  0x73); // Modem configuration.
    RFWriteRegister(MDMCFG1,  0x22); // Modem configuration.
    RFWriteRegister(MDMCFG0,  0xF8); // Modem configuration.
    RFWriteRegister(DEVIATN,  0x00); // Modem dev (when FSK mod en)
    RFWriteRegister(MCSM1 ,   0x3F); // MainRadio Cntrl State Machine
    RFWriteRegister(MCSM0 ,   0x18); // MainRadio Cntrl State Machine
    RFWriteRegister(FOCCFG,   0x1D); // Freq Offset Compens. Config
    RFWriteRegister(BSCFG,    0x1C); // Bit synchronization config.
    RFWriteRegister(AGCCTRL2, 0xC7); // AGC control.
    RFWriteRegister(AGCCTRL1, 0x00); // AGC control.
    RFWriteRegister(AGCCTRL0, 0xB2); // AGC control.
    RFWriteRegister(FREND1,   0xB6); // Front end RX configuration.
    RFWriteRegister(FREND0,   0x10); // Front end RX configuration.
    RFWriteRegister(FSCAL3,   0xEA); // Frequency synthesizer cal.
    RFWriteRegister(FSCAL2,   0x0A); // Frequency synthesizer cal.
    RFWriteRegister(FSCAL1,   0x00); // Frequency synthesizer cal.
    RFWriteRegister(FSCAL0,   0x11); // Frequency synthesizer cal.
    RFWriteRegister(FSTEST,   0x59); // Frequency synthesizer cal.
    RFWriteRegister(TEST2,    0x88); // Various test settings.
    RFWriteRegister(TEST1,    0x31); // Various test settings.
    RFWriteRegister(TEST0,    0x0B); // Various test settings.
    RFWriteRegister(PATABLE,  0xFE); // RF Power.
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void RFWriteRegister(char address, char data)
{
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = address;
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = data;
  while (UCB0STAT & UCBUSY);
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void WriteStrobe(char strobe)
{
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2&UCB0TXIFG));
  UCB0TXBUF = strobe;
  while (UCB0STAT & UCBUSY);
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char RFReadRegister(char address)
{
  char data;
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = (address | READ_BURST_BIT);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = 0;
  while (UCB0STAT & UCBUSY);
  data = UCB0RXBUF;
  P3OUT = P3OUT | PIN_CS_RF;
  return data;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void BurstReadRegister(char addr, char *buffer, char count)
{
  char aux;

  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2&UCB0TXIFG));
  UCB0TXBUF = (addr | READ_BURST_BIT);
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0;
  IFG2 = IFG2 & (~UCB0RXIFG);
  while (!(IFG2 & UCB0RXIFG));

  for (aux = 0; aux < (count - 1); aux++)
  {
    UCB0TXBUF = 0;
    buffer[aux] = UCB0RXBUF;
    while (!(IFG2 & UCB0RXIFG));
  }
  
  buffer[count - 1] = UCB0RXBUF;
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char RFReceivePacket(char *rxBuffer, char length)
{
  char status[2];
  char pktLen;

  if (RFReadRegister(RXBYTES) & NUM_RXBYTES)
  {
    pktLen = RFReadRegister(RXFIFO);
    BurstReadRegister(RXFIFO, rxBuffer, pktLen);
    BurstReadRegister(RXFIFO, status, 2);
    return 1;
  }
  else
  {
    return 0;
  }
}