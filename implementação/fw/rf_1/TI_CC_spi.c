#include "include.h"

//------------------------------------------------------------------------------
//  void TI_CC_Wait(unsigned int cycles)
//
//  DESCRIPTION:
//  Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
//  it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
//  delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
//------------------------------------------------------------------------------
void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

//------------------------------------------------------------------------------
//  void TI_CC_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//------------------------------------------------------------------------------
void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  UCB0CTL1 |= UCSWRST;                      // **Disable USCI state machine**
  UCB0CTL0 |= UCMST+UCCKPH+UCMSB+UCSYNC;    // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                     // SMCLK
  UCB0BR0 = 0x02;                           // UCLK/2
  UCB0BR1 = 0;
  TI_CC_SPI_USCIB0_PxSEL |= TI_CC_SPI_USCIB0_SIMO
                         | TI_CC_SPI_USCIB0_SOMI
                         | TI_CC_SPI_USCIB0_UCLK;
                                            // SPI option select
  TI_CC_SPI_USCIB0_PxDIR |= TI_CC_SPI_USCIB0_SIMO | TI_CC_SPI_USCIB0_UCLK;
                                            // SPI TXD out direction
  UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

//------------------------------------------------------------------------------
//  void TI_CC_SPIWriteReg(char addr, char value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//------------------------------------------------------------------------------
void TI_CC_SPIWriteReg(char addr, char value)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = addr;                         // Send address
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = value;                        // Send data
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

//------------------------------------------------------------------------------
//  void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//------------------------------------------------------------------------------
void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = addr | TI_CCxxx0_WRITE_BURST; // Send address
  for (i = 0; i < count; i++)
  {
    while (!(IFG2&UCB0TXIFG));              // Wait for TXBUF ready
    UCB0TXBUF = buffer[i];                  // Send data
  }
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

//------------------------------------------------------------------------------
//  char TI_CC_SPIReadReg(char addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//------------------------------------------------------------------------------
char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_SINGLE);// Send address
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = 0;                            // Dummy write so we can read data
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  x = UCB0RXBUF;                            // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

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

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
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
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

//------------------------------------------------------------------------------
//  char TI_CC_SPIReadStatus(char addr)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//------------------------------------------------------------------------------
char TI_CC_SPIReadStatus(char addr)
{
  char status;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST);// Send address
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = 0;                            // Dummy write so we can read data
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  status = UCB0RXBUF;                       // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return status;
}

//------------------------------------------------------------------------------
//  void TI_CC_SPIStrobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//------------------------------------------------------------------------------
void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = strobe;                       // Send strobe
  // Strobe addr is now being TX'ed
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (!(IFG2&UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = TI_CCxxx0_SRES;               // Send strobe
  // Strobe addr is now being TX'ed
  while (UCB0STAT & UCBUSY);                // Wait for TX to complete
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

//------------------------------------------------------------------------------
//  void writeRFSettings(void)
//
//  DESCRIPTION:
//  Used to configure the CCxxxx registers.  There are five instances of this
//  function, one for each available carrier frequency.  The instance compiled
//  is chosen according to the system variable TI_CC_RF_FREQ, assigned within
//  the header file "TI_CC_hardware_board.h".
//
//  ARGUMENTS:
//      none
//------------------------------------------------------------------------------
// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 0.000000
// Return state:  Return to RX state upon leaving either TX or RX
// Datarate = 250.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2433.000000 MHz
// Channel spacing = 199.950000 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock
void writeRFSettings(void)
{
    // Write register settings
    TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG2,   0x0B);  // GDO2 output pin config.
    TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG0,   0x06);  // GDO0 output pin config.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL1, 0x05);  // Packet automation control.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL0, 0x05);  // Packet automation control.
    TI_CC_SPIWriteReg(TI_CCxxx0_ADDR,     0x01);  // Device address.
    TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL1,  0x07); // Freq synthesizer control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ2,    0x5D); // Freq control word, high byte
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ1,    0x93); // Freq control word, mid byte.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ0,    0xB1); // Freq control word, low byte.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG4,  0x2D); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG3,  0x3B); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG2,  0x73); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG1,  0x22); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
    TI_CC_SPIWriteReg(TI_CCxxx0_MCSM1 ,   0x3F); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(TI_CCxxx0_MCSM0 ,   0x18); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(TI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
    TI_CC_SPIWriteReg(TI_CCxxx0_BSCFG,    0x1C); //  Bit synchronization config.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL2, 0xC7); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL1, 0x00); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL0, 0xB2); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST0,    0x0B);  // Various test settings.
}

//-----------------------------------------------------------------------------
//  void RFSendPacket(char *txBuffer, char size)
//
//  DESCRIPTION:
//  This function transmits a packet with length up to 63 bytes.  To use this
//  function, GD00 must be configured to be asserted when sync word is sent and
//  de-asserted at the end of the packet, which is accomplished by setting the
//  IOCFG0 register to 0x06, per the CCxxxx datasheet.  GDO0 goes high at
//  packet start and returns low when complete.  The function polls GDO0 to
//  ensure packet completion before returning.
//
//  ARGUMENTS:
//      char *txBuffer
//          Pointer to a buffer containing the data to be transmitted
//
//      char size
//          The size of the txBuffer
//-----------------------------------------------------------------------------
void RFSendPacket(char *txBuffer, char size)
{
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, size); // Write TX data
  TI_CC_SPIStrobe(TI_CCxxx0_STX);           // Change state to TX, initiating
                                            // data transfer

  while (!(TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN));
                                            // Wait GDO0 to go hi -> sync TX'ed
  while (TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN);
                                            // Wait GDO0 to clear -> end of pkt
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt TX, this flag is set.
                                            // Has to be cleared before existing
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

  if ((TI_CC_SPIReadStatus(TI_CCxxx0_RXBYTES) & TI_CCxxx0_NUM_RXBYTES))
  {
    pktLen = TI_CC_SPIReadReg(TI_CCxxx0_RXFIFO); // Read length byte

    if (pktLen <= *length)                  // If pktLen size <= rxBuffer
    {
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, rxBuffer, pktLen); // Pull data
      *length = pktLen;                     // Return the actual size
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, status, 2);
                                            // Read appended status bytes
      return (char)(status[TI_CCxxx0_LQI_RX]&TI_CCxxx0_CRC_OK);
    }                                       // Return CRC_OK bit
    else
    {
      *length = pktLen;                     // Return the large size
      TI_CC_SPIStrobe(TI_CCxxx0_SFRX);      // Flush RXFIFO
      return 0;                             // Error
    }
  }
  else
      return 0;                             // Error
}
