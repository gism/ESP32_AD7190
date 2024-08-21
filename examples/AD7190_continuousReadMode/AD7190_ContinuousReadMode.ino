
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
#define MAIN_DEBUG_VERBOSE           //  Print some more information

#define MEASURE_PIN 21

AD7190* ad7190 = NULL;

uint32_t rawAd7190Data = 0;
uint32_t AD7190_rawZero = 0x800000;        // Arbitrary number based on my setup
int32_t t = 0;

bool timeout = false;

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

void configureAd7190ContinuousRead(){
  
  // Set GPIOS required for development board:
  uint8_t regGPIOSettings = AD7190_GPOCON_GP32EN | AD7190_GPOCON_BPDSW;                                         //  Set excitation source  (P3 to low is 5V exitation on)
  ad7190->setRegisterValue(AD7190_REG_GPOCON, regGPIOSettings, 1, AD7190_CS_CHANGE);

  // Set AD7190 configuration
  // Channel #2 (AIN3 - AIN4): As debug board
  // Gain 128
  // No Buffer
  // Bipolar read
  // Chop active (ADC offset drift minimized)
  uint32_t regConfigSettings = (AD7190_CONF_CHOP | 
                                AD7190_CONF_CHAN(AD7190_CH_AIN3P_AIN4M) | 
                                AD7190_CONF_GAIN(AD7190_CONF_GAIN_128));
                                
  ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3, AD7190_CS_CHANGE);

  // Set AD7190 mode
  // Continuous conversion mode
  // SINC4 fitler (default)
  // Filter: 1
  uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_CONT) | 
                              AD7190_MODE_CLKSRC(AD7190_CLK_INT) | 
                              AD7190_MODE_RATE(AD7190_FILTER_RATE_1));
                              
  ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3, AD7190_CS_CHANGE);

  // Set Continuous Read Mode:
  uint8_t regCommSetting = (AD7190_COMM_READ | 
                            AD7190_COMM_ADDR(AD7190_REG_DATA) | 
                            AD7190_COMM_CREAD);
                            
  ad7190->setModeContinuousRead(regCommSetting);

  Serial.println("configureAd7190ContinuousRead DONE");
}

int32_t getAd7190SampleContinuousRead(){

    // Conversion time should take 1.67 miliseconds from Table 10 (page 17) of datasheet
    timeout = !ad7190->waitRdyGoLow();
    //if (timeout)Serial.println("RDY timeout");

    rawAd7190Data = ad7190->getDataContinuousRead(3);
    t = rawAd7190Data - AD7190_rawZero;

  // this takes about: 130 ms
  #ifdef MAIN_DEBUG_CONVERSION
      Serial.print("ts: ");
      Serial.print(millis());             // Print time stamp
      
      Serial.print(" RAW value: ");
      Serial.print(rawAd7190Data, HEX);         // Print Raw ADC value
      
      Serial.print(" --> ");
      Serial.println(t);
  #endif

  return t;
  
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("\n\nHola. This is AD7190 demo for ESP32\n\n");

  pinMode(MEASURE_PIN, OUTPUT);       //  Sample rate measurement PIN

  initAd7190();                       //  Initialize AD7190
  
  configureAd7190ContinuousRead();    //  Set AD7190 to Continuous Read Mode
  
}

void loop() {

  // My measurement is:
  // Freq:    ~605  Hz
  // Period:  1.650 ms
  
  // Meaning each sample is done in 0.825 ms
  
  // From Table 10 and 11: SINC4 CHOP ENABLE
  // Output Data Rate = 1200 Hz --> 0.833 ms
  // This is matching my measurement

  // Setting time is 1.67 ms
  
  digitalWrite(MEASURE_PIN, LOW);     //  TICK
  getAd7190SampleContinuousRead();
  
  digitalWrite(MEASURE_PIN, HIGH);     //  TACK
  getAd7190SampleContinuousRead();
  
}
