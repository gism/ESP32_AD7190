
//    Example how to use AD7190 with ESP32 board.
//    It's been tested with AD7190 strain acquisition module:
//    https://github.com/gism/ESP32_AD7190/blob/main/Documentation/AD7190_Test_Board.pdf
//    
//    Conection as follows:
//    
//    AD7190 board -- ESP32 board
//           GND   --  GND
//           SCLK  --  GPIO14
//           DIN   --  GPIO13 (MISO)
//           SYNC  --  Not connected
//       DOUT/RDY  --  GPIO12 (MOSI)
//           CS    --  GPIO15
//           VIN   --  3V3

#include <SPI.h>
#include "AD7190.h"

bool timeOut = false;
uint32_t rawAd7190Data = 0x0;
uint32_t sampleTime = 0x0;

AD7190* ad7190 = NULL;

void setup() {
  
  Serial.begin(115200);
  Serial.println("\n\nHola. This is AD7190 demo for ESP32\n\n");

  // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  
  SPIClass* spi = new SPIClass(HSPI);

  // Its also possible to use VSPI port
  // Initialise SPI Port V SPI with default pins:
  // SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  
  // SPIClass* spi = new SPIClass(VSPI);
  
  uint8_t hspi_mosi_pin = 12;  
  ad7190 = new AD7190(spi, hspi_mosi_pin, "Channel A");

  if(ad7190->begin()){
    Serial.println(F("AD7190 begin: OK"));
    Serial.print("Device name: ");
    Serial.println(ad7190->getDeviceName());
  }else{
    Serial.println(F("AD7190 begin: FAIL"));
  }
  
  float t = ad7190->getTemperature();

  Serial.print("Temperature from setup: ");
  Serial.println(t);

  // Set GPIOS required for development board:
  uint8_t regGPIOSettings = AD7190_GPOCON_GP32EN;                     //  Set excitation source  (P3 to low is 5V exitation on)
  ad7190->setRegisterValue(AD7190_REG_GPOCON, regGPIOSettings, 1);

  // Set AD7190 configuration
  uint32_t regConfigSettings = (AD7190_CONF_CHAN(AD7190_CH_AIN3P_AIN4M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF);
  ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3);

  // Set AD7190 mode
  uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_1));
  ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3);

  // Set Continuous Read Mode:
  uint8_t regCommSetting = (AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA) | AD7190_COMM_CREAD);
  ad7190->setModeContinuousRead(regCommSetting);
  
}


// The loop function runs over and over again until power down or reset
void loop() {

  timeOut = !ad7190->waitMisoGoLow();
  rawAd7190Data = ad7190->getDataRegisterAvg(3);

  if (timeOut)Serial.println("RDY time out");
  
  int32_t AD7190_rawZero = 0x7FD9F;
  int32_t t = rawAd7190Data - AD7190_rawZero;

  int32_t samplingTime = millis() - sampleTime;

  Serial.print(ad7190->getDeviceName());                     // Print time stamp

  Serial.print(" ts: ");
  Serial.print(millis());                     // Print time stamp

  Serial.print(" d: ");
  Serial.print(samplingTime);                 // Print time stamp
      
  Serial.print(" RAW value: ");
  Serial.print(rawAd7190Data, HEX);           // Print Raw ADC value
      
  Serial.print(" --> ");
  Serial.println(t);

  sampleTime = millis();
  
}
