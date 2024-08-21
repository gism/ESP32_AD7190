
//    Example how to use AD7190 with ESP32 board.
//    It's been tested with AD7190 strain acquisition module:
//    https://github.com/gism/ESP32_AD7190/blob/main/Documentation/AD7190_Test_Board.pdf
//    
//    Conection as follows:
//    
//    AD7190 board -- ESP32 board
//           GND   --  GND
//           SCLK  --  GPIO14
//           DIN   --  GPIO13 (COPI)
//           SYNC  --  Not connected
//       DOUT/RDY  --  GPIO12 (CIPO)
//           CS    --  GPIO15
//           VIN   --  3V3

#include <SPI.h>
#include <AD7190.h>

                                       // Debug prints levels:
#define MAIN_DEBUG_CONVERSION          //  Prints string at begining of class function execution
#define MAIN_DEBUG_VERBOSE             //  Print some more information

#define MEASURE_PIN 21
bool measurementPin = false;

AD7190* ad7190 = NULL;

uint32_t rawAd7190Data_Channel1 = 0;
uint32_t rawAd7190Data_Channel2 = 0;

bool initAd7190(){

  // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, CIPO = 12, COPI = 13, SS = 15
  
  SPIClass* spi = new SPIClass(HSPI);

  // Its also possible to use VSPI port
  // Initialise SPI Port V SPI with default pins:
  // SCLK = 18, CIPO = 19, COPI = 23, SS = 5
  
  // SPIClass* spi = new SPIClass(VSPI);
  
  uint8_t hspi_mosi_pin = 12;  
  ad7190 = new AD7190(spi, hspi_mosi_pin, "A");               // A stands for Channel A

  if(ad7190->begin()){
  #ifdef MAIN_DEBUG_VERBOSE
    Serial.println(F("AD7190 begin: OK"));
    Serial.print("Device name: ");
    Serial.println(ad7190->getDeviceName());
  #endif

    float t = ad7190->getTemperature();
    Serial.print(F("AD7190 Temperature: "));
    Serial.println(t);

    return true;
    
  #ifdef MAIN_DEBUG_VERBOSE
  }else{
  
    Serial.println(F("AD7190 begin: FAIL"));
  #endif
  }
  return false;
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("\n\nHola. This is AD7190 demo for ESP32\n\n");

  pinMode(MEASURE_PIN, OUTPUT);       //  Sample rate measurement PIN

  initAd7190();                       //  Initialize AD7190
  
  // Set excitation source  (P3 to low is 5V exitation on)
  // Activate P2 and P3
  // Close BPDSW switch
  uint8_t regGPIOSettings = (AD7190_GPOCON_BPDSW | 
                             AD7190_GPOCON_GP32EN |
                             AD7190_GPOCON_P2DAT | 
                             AD7190_GPOCON_P3DAT);

  ad7190->setRegisterValue(AD7190_REG_GPOCON, regGPIOSettings, 1, AD7190_CS_CHANGE);

}

void loop() {

  // Set AD7190 configuration
  // Channel #2 (AIN3 - AIN4): As debug board
  // Gain 128
  // No Buffer
  // Bipolar read
  // Chop active (ADC offset drift minimized)
  uint32_t regConfigSettings = (AD7190_CONF_CHOP | 
                                //AD7190_CONF_REFSEL |
                                //AD7190_CONF_BURN |
                                //AD7190_CONF_BUF |
                                AD7190_CONF_CHAN(AD7190_CH_AIN3P_AIN4M) | 
                                AD7190_CONF_GAIN(AD7190_CONF_GAIN_128));

  ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3, AD7190_CS_CHANGE);
  

  // Set AD7190 mode
  // Single conversion mode
  // No status register attached
  // Internal 4.92 MHz clock (Pin MCLK2 is tristated)
  // SINC4 fitler (default)
  // Filter: 1
  uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_SINGLE) | 
                              AD7190_MODE_CLKSRC(AD7190_CLK_INT) | 
                              //AD7190_MODE_CLKSRC(AD7190_CLK_EXT_MCLK1_2) | 
                              AD7190_MODE_RATE(AD7190_FILTER_RATE_1));

  ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3, AD7190_CS_CHANGE);

  // Wait AD7190 Convention to finish
  // this is indicated by asserting !RDY (CIPO/MISO) pin low
  ad7190->waitRdyGoLow();

  // Read channel 1 result
  rawAd7190Data_Channel1 = ad7190->getRegisterValue(AD7190_REG_DATA, 3, AD7190_CS_CHANGE);


  // Set AD7190 configuration
  // Channel #2 (AIN1 - AIN2)
  // Gain 128
  // No Buffer
  // Bipolar read
  // Chop active (ADC offset drift minimized)
  regConfigSettings = (AD7190_CONF_CHOP | 
                       //AD7190_CONF_REFSEL |
                       //AD7190_CONF_BURN |
                       //AD7190_CONF_BUF |
                       AD7190_CONF_CHAN(AD7190_CH_AIN1P_AIN2M) | 
                       AD7190_CONF_GAIN(AD7190_CONF_GAIN_128));

  ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3, AD7190_CS_CHANGE);
  
  // Set AD7190 mode
  // Single conversion mode
  // No status register attached
  // Internal 4.92 MHz clock (Pin MCLK2 is tristated)
  // SINC4 fitler (default)
  // Filter: 1
  regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_SINGLE) | 
                     AD7190_MODE_CLKSRC(AD7190_CLK_INT) | 
                     //AD7190_MODE_CLKSRC(AD7190_CLK_EXT_MCLK1_2) | 
                     AD7190_MODE_RATE(AD7190_FILTER_RATE_1));

  ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3, AD7190_CS_CHANGE);

  // Wait AD7190 Convention to finish
  // this is indicated by asserting !RDY (CIPO/MISO) pin low
  ad7190->waitRdyGoLow();

  // Read channel 2 result
  rawAd7190Data_Channel2 = ad7190->getRegisterValue(AD7190_REG_DATA, 3, AD7190_CS_CHANGE);


  // Toggle one GPIO each loop (2x channels measured)
  digitalWrite(MEASURE_PIN, measurementPin);     //  TICK, TACK each measurement
  measurementPin != measurementPin;

  #ifdef MAIN_DEBUG_CONVERSION
	  Serial.print(rawAd7190Data_Channel1, HEX);
    Serial.print('\t');
    Serial.println(rawAd7190Data_Channel2, HEX);
  #endif.

}
