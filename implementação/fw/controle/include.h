#include "msp430.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void SPIInitialization(void);
void RFInitialization(void);
void RFConfiguration(void);
void RFWriteRegister(char, char);
void WriteStrobe(char);
void BurstWriteRegister(char, char *, char);
char RFReadRegister(char);
void BurstReadRegister(char, char *, char);
char RFReceivePacket(char *, char);
unsigned char ReadRegister(unsigned char address);
unsigned char WriteRegister(unsigned char address, unsigned char data);
unsigned char RFSendData(unsigned char data);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Pins Definitions

#define PIN_MOSI    BIT1    // Porta 3
#define PIN_MISO    BIT2    // Porta 3
#define PIN_SCK     BIT3    // Porta 3
#define PIN_CS_RF   BIT0    // Porta 3
#define PIN_CS_ACC  BIT4    // Porta 4

#define PIN_LED1    BIT4    // Porta 2
#define PIN_LED2    BIT2    // Porta 2
#define PIN_LED3    BIT3    // Porta 2

#define PIN_GDO0    BIT6

//#############################################################################
//#############################################################################

// Accelerometer Registers

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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Strobe Commands

#define SRES        0x30        // Reset chip.
#define SFSTXON     0x31        // Enable/calibrate freq synthesizer
#define SXOFF       0x32        // Turn off crystal oscillator.
#define SCAL        0x33        // Calibrate freq synthesizer & disable
#define SRX         0x34        // Enable RX.
#define STX         0x35        // Enable TX.
#define SIDLE       0x36        // Exit RX / TX
#define SAFC        0x37        // AFC adjustment of freq synthesizer
#define SWOR        0x38        // Start automatic RX polling sequence
#define SPWD        0x39        // Enter pwr down mode when CSn goes hi
#define SFRX        0x3A        // Flush the RX FIFO buffer.
#define SFTX        0x3B        // Flush the TX FIFO buffer.
#define SWORRST     0x3C        // Reset real time clock.
#define SNOP        0x3D        // No operation.

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// RF Controller Configuration Registers

#define IOCFG2      0x00        // GDO2 output pin configuration
#define IOCFG1      0x01        // GDO1 output pin configuration
#define IOCFG0      0x02        // GDO0 output pin configuration
#define FIFOTHR     0x03        // RX FIFO and TX FIFO thresholds
#define SYNC1       0x04        // Sync word, high byte
#define SYNC0       0x05        // Sync word, low byte
#define PKTLEN      0x06        // Packet length
#define PKTCTRL1    0x07        // Packet automation control
#define PKTCTRL0    0x08        // Packet automation control
#define ADDR        0x09        // Device address
#define CHANNR      0x0A        // Channel number
#define FSCTRL1     0x0B        // Frequency synthesizer control
#define FSCTRL0     0x0C        // Frequency synthesizer control
#define FREQ2       0x0D        // Frequency control word, high byte
#define FREQ1       0x0E        // Frequency control word, middle byte
#define FREQ0       0x0F        // Frequency control word, low byte
#define MDMCFG4     0x10        // Modem configuration
#define MDMCFG3     0x11        // Modem configuration
#define MDMCFG2     0x12        // Modem configuration
#define MDMCFG1     0x13        // Modem configuration
#define MDMCFG0     0x14        // Modem configuration
#define DEVIATN     0x15        // Modem deviation setting
#define MCSM2       0x16        // Main Radio Cntrl State Machine config
#define MCSM1       0x17        // Main Radio Cntrl State Machine config
#define MCSM0       0x18        // Main Radio Cntrl State Machine config
#define FOCCFG      0x19        // Frequency Offset Compensation config
#define BSCFG       0x1A        // Bit Synchronization configuration
#define AGCCTRL2    0x1B        // AGC control
#define AGCCTRL1    0x1C        // AGC control
#define AGCCTRL0    0x1D        // AGC control
#define WOREVT1     0x1E        // High byte Event 0 timeout
#define WOREVT0     0x1F        // Low byte Event 0 timeout
#define WORCTRL     0x20        // Wake On Radio control
#define FREND1      0x21        // Front end RX configuration
#define FREND0      0x22        // Front end TX configuration
#define FSCAL3      0x23        // Frequency synthesizer calibration
#define FSCAL2      0x24        // Frequency synthesizer calibration
#define FSCAL1      0x25        // Frequency synthesizer calibration
#define FSCAL0      0x26        // Frequency synthesizer calibration
#define RCCTRL1     0x27        // RC oscillator configuration
#define RCCTRL0     0x28        // RC oscillator configuration
#define FSTEST      0x29        // Frequency synthesizer cal control
#define PTEST       0x2A        // Production test
#define AGCTEST     0x2B        // AGC test
#define TEST2       0x2C        // Various test settings
#define TEST1       0x2D        // Various test settings
#define TEST0       0x2E        // Various test settings

// RF Controller Status Registers

#define PARTNUM      0x30       // Part number
#define VERSION      0x31       // Current version number
#define FREQEST      0x32       // Frequency offset estimate
#define LQI          0x33       // Demodulator estimate for link quality
#define RSSI         0x34       // Received signal strength indication
#define MARCSTATE    0x35       // Control state machine state
#define WORTIME1     0x36       // High byte of WOR timer
#define WORTIME0     0x37       // Low byte of WOR timer
#define PKTSTATUS    0x38       // Current GDOx status and packet status
#define VCO_VC_DAC   0x39       // Current setting from PLL cal module
#define TXBYTES      0x3A       // Underflow and # of bytes in TXFIFO
#define RXBYTES      0x3B       // Overflow and # of bytes in RXFIFO
#define PATABLE      0x3E       // RF Power
#define TXFIFO       0x3F       // TX Buffer
#define RXFIFO       0x3F       // RX Buffer
#define NUM_RXBYTES  0x7F       // Mask "# of bytes" field in _RXBYTES

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Auxiliary Definitions

#define WRITE_BURST_BIT  0x40
#define READ_BURST_BIT   0xC0
#define READ_SINGLE      0x80