
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
#include "AD7190.h"

#include "config.h"
#include "espInfo.h"
#include "wifiTasks.h"

AD7190* ad7190 = NULL;

static QueueHandle_t ad7190DataQueueHandler = NULL;
char queuedData[MAX_MESSAGE_BYTES];

uint32_t rawAd7190Data = 0;
uint32_t AD7190_rawZero = 0x7FD8FF;        // Arbitrary number based on my setup
bool timeout = false;
uint32_t sampleCount = 0;

bool initAd7180(){

  // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, RDY/CIPO = 12, COPI = 13, SS = 15
  
  SPIClass* spi = new SPIClass(HSPI);

  // Its also possible to use VSPI port
  // Initialise SPI Port V SPI with default pins:
  // SCLK = 18, RDY/CIPO = 19, COPI = 23, SS = 5
  
  // SPIClass* spi = new SPIClass(VSPI);
  
  uint8_t hspi_rdy_pin = 12;  
  ad7190 = new AD7190(spi, hspi_rdy_pin, "A");               // A stands for Channel A

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
  uint8_t regGPIOSettings = AD7190_GPOCON_GP32EN;                                         //  Set excitation source  (P3 to low is 5V exitation on)
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

    //printf("getAd7190SampleContinuousRead %d!\n", xPortGetCoreID() );

    // Conversion time should take 1.67 miliseconds from Table 10 (page 17) of datasheet
    timeout = !ad7190->waitRdyGoLow();
    //if (timeout)Serial.println("RDY timeout");

    rawAd7190Data = ad7190->getDataContinuousRead(3);
    int32_t t = rawAd7190Data - AD7190_rawZero;

    // this takes about: 130 ms
  #ifdef MAIN_DEBUG_VERBOSE
      Serial.print("ts: ");
      Serial.print(millis());             // Print time stamp
      
      Serial.print(" RAW value: ");
      Serial.print(rawAd7190Data, HEX);         // Print Raw ADC value
      
      Serial.print(" --> ");
      Serial.println(t);
  #endif

  return t;
  
}

void taskProcessAd7190Data( void *pvParameters ) {

#ifdef MAIN_DEBUG_CALLS
  Serial.println(F("Task Started: taskProcessAd7190Data"));
#endif
  

  while (1) {
    xQueueReceive(ad7190DataQueueHandler, &queuedData, portMAX_DELAY);         // Setting xTicksToWait to portMAX_DELAY will cause the task to wait indefinitely (without timing out)

    if (!serverIsConnected()) {
      connectServer();
    }else{
      streamAd7190DataMessageToServer(queuedData);
    }
    
  }
}

void queueAd7190Data(int32_t w) {
  
  char dataQueuedMessage[MAX_MESSAGE_BYTES];
  memset(dataQueuedMessage, 0, sizeof(dataQueuedMessage));
  sprintf(dataQueuedMessage, "n:%s t:%d c:%d v:%d\n", ad7190->getDeviceName(), millis(), sampleCount, w);

  sampleCount = sampleCount + 1;

  if(!uxQueueSpacesAvailable(ad7190DataQueueHandler)){
     //xQueueReset(ad7190DataQueueHandler);
     //Serial.println(F("RESET ad7190DataQueueHandler"));

    char queuedData[MAX_MESSAGE_BYTES];
    for (uint8_t i = 0; i <= MAX_QUEUE_LENGTH/3; i++) {
      xQueueReceive(ad7190DataQueueHandler, &queuedData, portMAX_DELAY);
    }
    Serial.println(F("DROP"));
  }
    
  xQueueSend(ad7190DataQueueHandler, &dataQueuedMessage, (TickType_t) 100);
  
  return;
}

void taskFetchAd7190Data( void *pvParameters ) {

#ifdef MAIN_DEBUG_CALLS
  Serial.println(F("Task Started: taskFetchAd7190Data"));
#endif

  char data[MAX_MESSAGE_BYTES];
  int32_t t = 0;
  bool ledStatus = false;
  
  while (1) {

    ledStatus = !ledStatus;
    
    digitalWrite(MEASURE_PIN, ledStatus);     //  TICK, TACK
    t = getAd7190SampleContinuousRead();
  
    queueAd7190Data(t);
    
  #ifdef MAIN_DEBUG_VERBOSE
    Serial.print("ts: ");
    Serial.print(millis());             // Print time stamp
    
    Serial.print(" RAW value: ");
    Serial.print(rawAd7190Data, HEX);         // Print Raw ADC value
    
    Serial.print(" --> ");
    Serial.println(t);

  #endif
    
  }
}

void initAd7190Tasks() {

#ifdef MAIN_DEBUG_CALLS
  Serial.println(F("AD7190_initTasks()"));
#endif
  
  ad7190DataQueueHandler = xQueueCreate(MAX_QUEUE_LENGTH, MAX_MESSAGE_BYTES);
  
  if (ad7190DataQueueHandler == 0) {
    #ifdef MAIN_DEBUG_VERBOSE
      Serial.println(F("Error creating ad7190DataQueueHandler"));
    #endif
  }

  xTaskCreatePinnedToCore(
    taskProcessAd7190Data,                            /* Function to implement the task */
    "taskProcessAd7190Data",                          /* Name of the task */
    2048,                                             /* Stack size in words. This stack size can be checked & adjusted by reading the Stack Highwater */
    NULL,                                             /* Task input parameter */
    3,                                                /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
    NULL,                                             /* Task handle. */
    0);                                               /* Core where the task should run */

#ifdef MAIN_DEBUG_VERBOSE
  Serial.println(F("Task created: taskProcessAd7190Data"));
#endif

  xTaskCreatePinnedToCore(
    taskFetchAd7190Data,                            /* Function to implement the task */
    "taskFetchAd7190Data",                          /* Name of the task */
    3072,                                           /* Stack size in words. This stack size can be checked & adjusted by reading the Stack Highwater */
    NULL,                                           /* Task input parameter */
    3,                                              /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
    NULL,                                           /* Task handle. */
    1);                                             /* Core where the task should run */

#ifdef MAIN_DEBUG_VERBOSE
  Serial.print(F("Task created: taskProcessAd7190Data"));
#endif

  return;
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("\n\nHola. This is AD7190 demo for ESP32\n\n");

  pinMode(MEASURE_PIN, OUTPUT);       //  Sample rate measurement PIN
  
  printBuildInfo();                   //  Print ESP32 and sketch information. Just debug info

  initAd7180();                       //  Initialize AD7190
  configureAd7190ContinuousRead();    //  Set AD7190 to Continuous Read Mode

  configureWifi();              //  Launches WifiManager with captive portal
  connectServer();              //  Connect to a hardcoded IP to stream AD7190 data

  initAd7190Tasks(); 
  
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

  // You can see many RESET/DROP ad7190DataQueueHandler on Serial
  // Every 25 samples
  // ESP32 sampling rate is faster than sample print rate
  // (taskFetchAd7190Data is faster than taskProcessAd7190Data)
  // if Serial speed is increased it is not shown

  vTaskDelay(10000);                  // Do nothing in main loop
  getServerMessage();
  
}
