
//    Hello together
//    from http://www.github.com/gism

//    Basic library for analog Devices AD7190
//    AD7190: 4.8 kHz Ultra-Low Noise 24-Bit Sigma-Delta ADC with PGA


#ifndef AD7190_H
#define AD7190_H

#include <Arduino.h>
#include <SPI.h>

#define AD7190_DOUT_TIMEOUT 100     //  TODO: This is arbitrary. Needs tuning

                                    // Debug prints levels:
//#define AD7190_DEBUG_CALLS          //  Prints string at begining of class function execution
//#define AD7190_DEBUG_VERBOSE        //  Print some more information

/* AD7190 Register Map */
#define AD7190_REG_COMM         0 // Communications Register (WO, 8-bit) 
#define AD7190_REG_STAT         0 // Status Register         (RO, 8-bit) 
#define AD7190_REG_MODE         1 // Mode Register           (RW, 24-bit) 
#define AD7190_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7190_REG_DATA         3 // Data Register           (RO, 24/32-bit) 
#define AD7190_REG_ID           4 // ID Register             (RO, 8-bit) 
#define AD7190_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit) 
#define AD7190_REG_OFFSET       6 // Offset Register         (RW, 24-bit 
#define AD7190_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD7190_REG_COMM) */
#define AD7190_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7190_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7190_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7190_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7190_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/* Status Register Bit Designations (AD7190_REG_STAT) */
#define AD7190_STAT_RDY         (1 << 7) // Ready.
#define AD7190_STAT_ERR         (1 << 6) // ADC error bit.
#define AD7190_STAT_NOREF       (1 << 5) // Error no external reference. 
#define AD7190_STAT_PARITY      (1 << 4) // Parity check of the data register. 
#define AD7190_STAT_CH2         (1 << 2) // Channel 2. 
#define AD7190_STAT_CH1         (1 << 1) // Channel 1. 
#define AD7190_STAT_CH0         (1 << 0) // Channel 0. 

/* Mode Register Bit Designations (AD7190_REG_MODE) */
#define AD7190_MODE_SEL(x)      (((x) & 0x7) << 21) // Operation Mode Select.
#define AD7190_MODE_DAT_STA     (1 << 20)           // Status Register transmission.
#define AD7190_MODE_CLKSRC(x)   (((x) & 0x3) << 18)  // Clock Source Select.
#define AD7190_MODE_SINC3       (1 << 15)           // SINC3 Filter Select.
#define AD7190_MODE_ENPAR       (1 << 13)           // Parity Enable.
#define AD7190_MODE_SCYCLE      (1 << 11)           // Single cycle conversion.
#define AD7190_MODE_REJ60       (1 << 10)           // 50/60Hz notch filter.
#define AD7190_MODE_RATE(x)     ((x) & 0x3FF)       // Filter Update Rate Select.

/* Mode Register: AD7190_MODE_SEL(x) options */
#define AD7190_MODE_CONT                0 // Continuous Conversion Mode.
#define AD7190_MODE_SINGLE              1 // Single Conversion Mode.
#define AD7190_MODE_IDLE                2 // Idle Mode.
#define AD7190_MODE_PWRDN               3 // Power-Down Mode.
#define AD7190_MODE_CAL_INT_ZERO        4 // Internal Zero-Scale Calibration.
#define AD7190_MODE_CAL_INT_FULL        5 // Internal Full-Scale Calibration.
#define AD7190_MODE_CAL_SYS_ZERO        6 // System Zero-Scale Calibration.
#define AD7190_MODE_CAL_SYS_FULL        7 // System Full-Scale Calibration.

/* Mode Register: AD7190_MODE_CLKSRC(x) options */
#define AD7190_CLK_EXT_MCLK1_2          0 // External crystal. The external crystal is connected from MCLK1 to MCLK2.
#define AD7190_CLK_EXT_MCLK2            1 // External Clock applied to MCLK2 
#define AD7190_CLK_INT                  2 // Internal 4.92 MHz clock. Pin MCLK2 is tristated.
#define AD7190_CLK_INT_CO               3 // Internal 4.92 MHz clock. The internal clock is available on MCLK2.

/* Filter Rate (AD7190_MODE_RATE)*/
/* Output Data Rate = (fmod/64)/FS */
#define AD7190_FILTER_RATE_1023   1023
#define AD7190_FILTER_RATE_640    640
#define AD7190_FILTER_RATE_480    480
#define AD7190_FILTER_RATE_96     96
#define AD7190_FILTER_RATE_80     80
#define AD7190_FILTER_RATE_32     32
#define AD7190_FILTER_RATE_16     16
#define AD7190_FILTER_RATE_5      5
#define AD7190_FILTER_RATE_2      2
#define AD7190_FILTER_RATE_1      1

/* Configuration Register Bit Designations (AD7190_REG_CONF) */
#define AD7190_CONF_CHOP        (1 << 23)            // CHOP enable.
#define AD7190_CONF_REFSEL      (1 << 20)            // REFIN1/REFIN2 Reference Select.
#define AD7190_CONF_CHAN(x)     (((x) & 0xFF) << 8)  // Channel select.
#define AD7190_CONF_BURN        (1 << 7)             // Burnout current enable.
#define AD7190_CONF_REFDET      (1 << 6)             // Reference detect enable.
#define AD7190_CONF_BUF         (1 << 4)             // Buffered Mode Enable.
#define AD7190_CONF_UNIPOLAR    (1 << 3)             // Unipolar/Bipolar Enable.
#define AD7190_CONF_GAIN(x)     ((x) & 0x7)          // Gain Select.

/* Configuration Register: AD7190_CONF_CHAN(x) options */
#define AD7190_CH_AIN1P_AIN2M      (1 << 0) // AIN1(+) - AIN2(-)       
#define AD7190_CH_AIN3P_AIN4M      (1 << 1) // AIN3(+) - AIN4(-)       
#define AD7190_CH_TEMP_SENSOR      (1 << 2) // Temperature sensor       
#define AD7190_CH_AIN2P_AIN2M      (1 << 3) // AIN2(+) - AIN2(-)       
#define AD7190_CH_AIN1P_AINCOM     (1 << 4) // AIN1(+) - AINCOM       
#define AD7190_CH_AIN2P_AINCOM     (1 << 5) // AIN2(+) - AINCOM       
#define AD7190_CH_AIN3P_AINCOM     (1 << 6) // AIN3(+) - AINCOM       
#define AD7190_CH_AIN4P_AINCOM     (1 << 7) // AIN4(+) - AINCOM

/* Configuration Register: AD7190_CONF_GAIN(x) options */
                                // ADC Input Range (5 V Reference)
#define AD7190_CONF_GAIN_1    0 // Gain 1    +-5 V
#define AD7190_CONF_GAIN_8    3 // Gain 8    +-625 mV
#define AD7190_CONF_GAIN_16   4 // Gain 16   +-312.5 mV
#define AD7190_CONF_GAIN_32   5 // Gain 32   +-156.2 mV
#define AD7190_CONF_GAIN_64   6 // Gain 64   +-78.125 mV
#define AD7190_CONF_GAIN_128  7 // Gain 128  +-39.06 mV
#define AD7190_CONF_GAIN_MASK 7 // To read gain config

#define AD7190_BUFF_ACTIVE     1
#define AD7190_BUFF_DISABLE    0

#define AD7190_UNIPOLAR        1
#define AD7190_BIPOLAR         0

/* ID Register Bit Designations (AD7190_REG_ID) */
#define ID_AD7190               0x4
#define AD7190_ID_MASK          0x07

/* GPOCON Register Bit Designations (AD7190_REG_GPOCON) */
#define AD7190_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD7190_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD7190_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD7190_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD7190_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD7190_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD7190_GPOCON_P0DAT     (1 << 0) // P0 state


//     AD7190 Settings:
//     SPI frequency: MAX AD7190 f is 5000000 = 5 MHz = (100 + 100) ns
//     SPI_MODE3 <-- CPOL = 1 CPHA = 1

static const int AD7190_SPI_CLK = 5000000;
static SPISettings AD7190_SPI_SETTINGS(AD7190_SPI_CLK, MSBFIRST, SPI_MODE3);

class AD7190 {
private:
    SPIClass* spiClass;
    char deviceName[16];
    uint8_t pin_rdy;

    boolean activeState;
    uint32_t spiErrorCount;
        
  #ifdef AD7190_DEBUG_VERBOSE
    char* getAddressDebugString(uint8_t a); 
  #endif

  
public:
    AD7190(SPIClass* spiClass, uint8_t pin_rdy);
    AD7190(SPIClass* spiClass, uint8_t pin_rdy, char* deviceName);
    ~AD7190();

    bool begin();
    void reset();
    bool checkId();

    char* getDeviceName();
    float getTemperature();
    bool waitMisoGoLow(void);
    
    // Generic read/write functions:
    uint32_t getRegisterValue(byte registerAddress, uint8_t bytesNumber);
    void setRegisterValue(unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber);

    // Custom read/write functions:
    uint8_t getStatusRegister();
    uint32_t getDataRegister(uint8_t sampleNumber);           // Checks status register and increas error counter
    uint32_t getDataRegisterAvg(uint8_t sampleNumber);        // Doesn't check status register, Doesn't increas error counter
    void setConfigurationRegister(uint8_t channel, uint8_t buff, uint8_t polarity, uint8_t range);
    
};


#endif
