#include "include.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SPIInitialization(void)
{
  P3OUT = P3OUT | PIN_CS_RF;
  P3DIR = P3DIR | PIN_CS_RF;
  
  UCB0CTL1 = UCB0CTL1 | UCSWRST;
  UCB0CTL0 = UCB0CTL0 | UCMST | UCCKPH | UCMSB | UCSYNC;
  UCB0CTL1 = UCB0CTL1 | UCSSEL_2;
  UCB0BR0 = 0x02;
  UCB0BR1 = 0;
  
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
  
  while (!(IFG2&UCB0TXIFG));
  UCB0TXBUF = SRES;
  while (UCB0STAT & UCBUSY);
  
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void RFConfiguration(void)
{
    WriteRegister(IOCFG2,   0x0B); // GDO2 output pin config.
    WriteRegister(IOCFG0,   0x06); // GDO0 output pin config.
    WriteRegister(PKTLEN,   0xFF); // Packet length.
    WriteRegister(PKTCTRL1, 0x05); // Packet automation control.
    WriteRegister(PKTCTRL0, 0x05); // Packet automation control.
    WriteRegister(ADDR,     0x01); // Device address.
    WriteRegister(CHANNR,   0x00); // Channel number.
    WriteRegister(FSCTRL1,  0x07); // Freq synthesizer control.
    WriteRegister(FSCTRL0,  0x00); // Freq synthesizer control.
    WriteRegister(FREQ2,    0x5D); // Freq control word, high byte
    WriteRegister(FREQ1,    0x93); // Freq control word, mid byte.
    WriteRegister(FREQ0,    0xB1); // Freq control word, low byte.
    WriteRegister(MDMCFG4,  0x2D); // Modem configuration.
    WriteRegister(MDMCFG3,  0x3B); // Modem configuration.
    WriteRegister(MDMCFG2,  0x73); // Modem configuration.
    WriteRegister(MDMCFG1,  0x22); // Modem configuration.
    WriteRegister(MDMCFG0,  0xF8); // Modem configuration.
    WriteRegister(DEVIATN,  0x00); // Modem dev (when FSK mod en)
    WriteRegister(MCSM1 ,   0x3F); // MainRadio Cntrl State Machine
    WriteRegister(MCSM0 ,   0x18); // MainRadio Cntrl State Machine
    WriteRegister(FOCCFG,   0x1D); // Freq Offset Compens. Config
    WriteRegister(BSCFG,    0x1C); // Bit synchronization config.
    WriteRegister(AGCCTRL2, 0xC7); // AGC control.
    WriteRegister(AGCCTRL1, 0x00); // AGC control.
    WriteRegister(AGCCTRL0, 0xB2); // AGC control.
    WriteRegister(FREND1,   0xB6); // Front end RX configuration.
    WriteRegister(FREND0,   0x10); // Front end RX configuration.
    WriteRegister(FSCAL3,   0xEA); // Frequency synthesizer cal.
    WriteRegister(FSCAL2,   0x0A); // Frequency synthesizer cal.
    WriteRegister(FSCAL1,   0x00); // Frequency synthesizer cal.
    WriteRegister(FSCAL0,   0x11); // Frequency synthesizer cal.
    WriteRegister(FSTEST,   0x59); // Frequency synthesizer cal.
    WriteRegister(TEST2,    0x88); // Various test settings.
    WriteRegister(TEST1,    0x31); // Various test settings.
    WriteRegister(TEST0,    0x0B); // Various test settings.
    WriteRegister(PATABLE,  0xFE); // RF Power.
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void WriteRegister(char address, char data)
{
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2&UCB0TXIFG));
  UCB0TXBUF = address;
  while (!(IFG2&UCB0TXIFG));
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

void BurstWriteRegister(char address, char *buffer, char count)
{
  unsigned int aux;
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2 & UCB0TXIFG));
  UCB0TXBUF = address | WRITE_BURST_BIT;
  
  for (aux = 0; aux < count; aux++)
  {
    while (!(IFG2 & UCB0TXIFG));
    UCB0TXBUF = buffer[aux];
  }
  
  while (UCB0STAT & UCBUSY);
  P3OUT = P3OUT | PIN_CS_RF;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char ReadRegister(char address)
{
  char data;
  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2 & UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (address | READ_BURST_BIT);      // Send address
  while (!(IFG2 & UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = 0;                            // Dummy write so we can read data
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  data = UCB0RXBUF;                            // Read data
  P3OUT = P3OUT | PIN_CS_RF;
  return data;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
//  void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//------------------------------------------------------------------------------
void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;

  P3OUT = P3OUT & (~PIN_CS_RF);
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (addr | READ_BURST_BIT);// Send address
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  UCB0TXBUF = 0;                            // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  IFG2 &= ~UCB0RXIFG;                       // Clear flag
  while (!(IFG2&UCB0RXIFG));                // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB0TXBUF = 0;                          //Initiate next data RX, meanwhile..
    buffer[i] = UCB0RXBUF;                  // Store data from last data RX
    while (!(IFG2&UCB0RXIFG));              // Wait for RX to finish
  }
  buffer[count-1] = UCB0RXBUF;              // Store last RX byte in buffer
  P3OUT = P3OUT | PIN_CS_RF;
}

//-----------------------------------------------------------------------------
//  char RFReceivePacket(char *rxBuffer, char *length)
//
//  DESCRIPTION:
//  Receives a packet of variable length (first byte in the packet must be the
//  length byte).  The packet length should not exceed the RXFIFO size.  To use
//  this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
//  is assumed that the function is called after it is known that a packet has
//  been received; for example, in response to GDO0 going low when it is
//  configured to output packet reception status.
//
//  The RXBYTES register is first read to ensure there are bytes in the FIFO.
//  This is done because the GDO signal will go high even if the FIFO is flushed
//  due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      char *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      char *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  RETURN VALUE:
//      char
//          0x80:  CRC OK
//          0x00:  CRC NOT OK (or no pkt was put in the RXFIFO due to filtering)
//-----------------------------------------------------------------------------
char RFReceivePacket(char *rxBuffer, char *length)
{
  char status[2];
  char pktLen;

  if ((ReadRegister(RXBYTES) & NUM_RXBYTES))
  {
    pktLen = ReadRegister(RXFIFO); // Read length byte

    if (pktLen <= *length)                  // If pktLen size <= rxBuffer
    {
      TI_CC_SPIReadBurstReg(RXFIFO, rxBuffer, pktLen); // Pull data
      *length = pktLen;                     // Return the actual size
      TI_CC_SPIReadBurstReg(RXFIFO, status, 2);
                                            // Read appended status bytes
      return (char)(status[TI_CCxxx0_LQI_RX]&TI_CCxxx0_CRC_OK);
    }                                       // Return CRC_OK bit
    else
    {
      *length = pktLen;                     // Return the large size
      WriteStrobe(SFRX);      // Flush RXFIFO
      return 0;                             // Error
    }
  }
  else
      return 0;                             // Error
}