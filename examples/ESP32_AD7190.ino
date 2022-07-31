
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
#include <AD7190.h>

// #define MAIN_DEBUG_CALLS          //  Prints string at begining of class function execution
// #define MAIN_DEBUG_VERBOSE        //  Print some more information

#define MAX_QUEUE_LENGTH               30
#define MAX_MESSAGE_BYTES              64
static QueueHandle_t ad7190DataQueueHandler = NULL;

#define SAMPLE_DELAY_INTERVAL     1

bool ledStatus = false;
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
  #ifdef MAIN_DEBUG_VERBOSE
    Serial.println(F("AD7190 begin: OK"));
    Serial.print("Device name: ");
    Serial.println(ad7190->getDeviceName());
  #endif
  }else{
  #ifdef MAIN_DEBUG_VERBOSE
    Serial.println(F("AD7190 begin: FAIL"));
  #endif
  }
  
  float t = ad7190->getTemperature();

  Serial.print("Temperature from setup: ");
  Serial.println(t);

  // Schedule tasks: 
  initAd7190Tasks();
}

// The loop function runs over and over again until power down or reset
void loop() {
  delay(1000);        //  Refresh watchdog?
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
    2,                                                /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
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
    2,                                              /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
    NULL,                                           /* Task handle. */
    0);                                             /* Core where the task should run */

#ifdef MAIN_DEBUG_VERBOSE
  Serial.print(F("Task created: taskProcessAd7190Data"));
#endif

}

void taskFetchAd7190Data( void *pvParameters ) {

#ifdef MAIN_DEBUG_CALLS
  Serial.println(F("Task Started: taskFetchAd7190Data"));
#endif

  char data[MAX_MESSAGE_BYTES];
  uint32_t sampleCount = 0;

  uint32_t rawAd7190Data = 0;
  
  while (1) {

    uint8_t regGPIOSettings = AD7190_GPOCON_GP32EN;         //  Set excitation source  (P3 to low is 5V exitation on)
    
    // Make LED to blink
    if(ledStatus) regGPIOSettings = regGPIOSettings | AD7190_GPOCON_P2DAT;
    ledStatus = !ledStatus;
    ad7190->setRegisterValue(AD7190_REG_GPOCON, regGPIOSettings, 1);

    uint32_t regConfigSettings = (AD7190_CONF_CHAN(AD7190_CH_AIN3P_AIN4M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF);   
    ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3);

    uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));
    ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3);

    ad7190->waitMisoGoLow();

    uint32_t rawAd7190Data = ad7190->getDataRegister(1);
      
    if (rawAd7190Data) {
      ++sampleCount;

      int32_t AD7190_rawZero = 0x7FD9F;
      int32_t t = rawAd7190Data - AD7190_rawZero;

      queueAd7190Data((float) t);

  #ifdef MAIN_DEBUG_VERBOSE
      Serial.print("ts: ");
      Serial.print(millis());             // Print time stamp
      
      Serial.print(" RAW value: ");
      Serial.print(rawAd7190Data, HEX);         // Print Raw ADC value
      
      Serial.print(" --> ");
      Serial.println(t);

      if (sampleCount % 1000 == 0) {
        Serial.print("AD7190 sample count: ");
        Serial.println(sampleCount);
      }
    #endif
      
    }
    //delay(SAMPLE_DELAY_INTERVAL);         // Delay in ms
  }
}

void queueAd7190Data(float w) {
  
  char dataQueuedMessage[MAX_MESSAGE_BYTES];
  memset(dataQueuedMessage, 0, sizeof(dataQueuedMessage));
  sprintf(dataQueuedMessage, "%s ts: %d v: %.2f", ad7190->getDeviceName(), millis(), w);

  xQueueSend(ad7190DataQueueHandler, &dataQueuedMessage, (TickType_t) 100);
}

void taskProcessAd7190Data( void *pvParameters ) {

#ifdef MAIN_DEBUG_CALLS
  Serial.println(F("Task Started: taskProcessAd7190Data"));
#endif
  
  char queuedData[MAX_MESSAGE_BYTES];

  while (1) {
    xQueueReceive(ad7190DataQueueHandler, &queuedData, portMAX_DELAY);         // Setting xTicksToWait to portMAX_DELAY will cause the task to wait indefinitely (without timing out)
    processAd7190DataMessage(queuedData);
  }
}

bool processAd7190DataMessage(char* messageBlock) {
  
  if (!messageBlock) {
  #ifdef MAIN_DEBUG_VERBOSE
    Serial.println(F("Error: processAd7190DataMessage, messageBlock is NULL"));
  #endif
    return false;
  }

  uint8_t n = strlen(messageBlock);
  Serial.println(messageBlock);
  
  return true;
}
