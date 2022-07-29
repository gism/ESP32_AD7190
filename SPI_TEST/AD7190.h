
#define AD7190_DEBUG

static const int AD7190_SPI_CLK = 10000;   // SE TIENE QUE CAMBIAR A 1000000 // 10 MHz  
static SPISettings AD7190_SPI_SETTINGS(AD7190_SPI_CLK, MSBFIRST, SPI_MODE3);


/* Communications Register Bit Designations (AD7190_REG_COMM) */
#define AD7190_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7190_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7190_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7190_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7190_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

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
//                                             ADC Input Range (5 V Reference)
#define AD7190_CONF_GAIN_1    0 // Gain 1    +-5 V
#define AD7190_CONF_GAIN_8    3 // Gain 8    +-625 mV
#define AD7190_CONF_GAIN_16   4 // Gain 16   +-312.5 mV
#define AD7190_CONF_GAIN_32   5 // Gain 32   +-156.2 mV
#define AD7190_CONF_GAIN_64   6 // Gain 64   +-78.125 mV
#define AD7190_CONF_GAIN_128  7 // Gain 128  +-39.06 mV
#define AD7190_CONF_GAIN_MASK 7 // To read gain config

/* ID Register Bit Designations (AD7190_REG_ID) */
#define ID_AD7190               0x4
#define AD7190_ID_MASK          0x07
