#include <SPI.h>
#include "AD7190.h"
           
#define SPI_MISO 12

SPIClass * ad7190 = NULL;       // Uninitalised pointer to SPI objects

#define MAX_QUEUE_LENGTH          30
#define MAX_MESSAGE_BYTES         64
static QueueHandle_t AD7190_spiQueue = NULL;

#define SAMPLE_DELAY_INTERVAL     1




AD7190* ad7190_myClass = NULL;


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Hola. This is AD7190 demo for ESP32");
  
  ad7190 = new SPIClass(HSPI);
  AD7190_begin(ad7190);

  AD7190_readTemperature(ad7190);
  
  AD7190_initTasks();



  //AD7190 ad7190_myClass(ad7190, "holaJulay!");
  ad7190_myClass = new AD7190(ad7190, "holaJulay!");

}

// The loop function runs over and over again until power down or reset
void loop() {
  delay(1000);        //  Refresh watchdog?
}






void AD7190_initTasks() {
  
  AD7190_spiQueue = xQueueCreate(MAX_QUEUE_LENGTH, MAX_MESSAGE_BYTES);
  
  if (AD7190_spiQueue == 0) {
    Serial.println(F("Error creating the AD7190_spiQueue"));
  }

  xTaskCreatePinnedToCore(
    taskProcessAd7190Data,                            /* Function to implement the task */
    "taskProcessAd7190Data",                          /* Name of the task */
    2048,                                         /* Stack size in words. This stack size can be checked & adjusted by reading the Stack Highwater */
    NULL,                                         /* Task input parameter */
    2,                                            /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
    NULL,                                         /* Task handle. */
    0);                                           /* Core where the task should run */

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("Task created: taskProcessAd7190Data"));
#endif

  xTaskCreatePinnedToCore(
    taskFetchAd7190Data,                            /* Function to implement the task */
    "taskFetchAd7190Data",                          /* Name of the task */
    3072,                                         /* Stack size in words. This stack size can be checked & adjusted by reading the Stack Highwater */
    NULL,                                         /* Task input parameter */
    2,                                            /* Priority of the task. With 3 (configMAX_PRIORITIES - 1) is the highest, 0 is the lowest. */
    NULL,                                         /* Task handle. */
    0);                                           /* Core where the task should run */

#ifdef AD7190_DEBUG_CALLS
  Serial.print(F("Task created: taskProcessAd7190Data"));
#endif

}

void taskProcessAd7190Data( void *pvParameters ) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("Task Started: taskProcessAd7190Data"));
#endif
  
  char queuedData[MAX_MESSAGE_BYTES];

  while (1) {
    xQueueReceive(AD7190_spiQueue, &queuedData, portMAX_DELAY);         // Setting xTicksToWait to portMAX_DELAY will cause the task to wait indefinitely (without timing out)
    sendAd7190Data(queuedData);
  }
}

void sendAd7190Data(char* toSendData) {
  
  if (!toSendData) {
    Serial.println(F("Error: sendAd7190Data, toSendData is NULL"));
    return;
  }

  uint8_t n = strlen(toSendData);
  Serial.print("sendAd7190Data: ");
  Serial.println(toSendData);

}


bool ledStatus = false;


void taskFetchAd7190Data( void *pvParameters ) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("Task Started: taskFetchAd7190Data"));
#endif

  char data[MAX_MESSAGE_BYTES];
  uint32_t sampleCount = 0;

  AD7190_begin(ad7190);
  
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  
  while (1) {

  // AD7190_setRegisterValue(SPIClass *spi, unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber) {

    uint8_t regGPIOSettings = AD7190_GPOCON_GP32EN;         //  Set excitation source  (P3 to low is 5V exitation on)
    
    // Make LED to blink
    if(ledStatus) regGPIOSettings = regGPIOSettings | AD7190_GPOCON_P2DAT;
    ledStatus = !ledStatus;
    AD7190_setRegisterValue(ad7190, AD7190_REG_GPOCON, regGPIOSettings, 1);

    uint32_t regConfigSettings = (AD7190_CONF_CHAN(AD7190_CH_AIN3P_AIN4M) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_128) | AD7190_CONF_BUF);   
    AD7190_setRegisterValue(ad7190, AD7190_REG_CONF, regConfigSettings, 3);

    uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));
    AD7190_setRegisterValue(ad7190, AD7190_REG_MODE, regModeSettings, 3);

    AD7190_waitMisoGoLow();
        
    if (AD7190_getData(ad7190)) {
      ++sampleCount;

    #ifdef AD7190_DEBUG_CALLS
      if (sampleCount % 1000 == 0) {
        Serial.print("AD7190 sample count: ");
        Serial.println(sampleCount);
      }
    #endif
      
    }
    delay(SAMPLE_DELAY_INTERVAL);         // Delay in ms
  }
}


boolean AD7190_activeState;
uint32_t Ad7190SpiErrorCount;
uint32_t AD7190_rawZero;
uint32_t AD7190_rawFactor;

boolean AD7190_getData(SPIClass *spi) {
  
  if (!AD7190_activeState) {
    ++Ad7190SpiErrorCount;
    
    if (Ad7190SpiErrorCount % 100 == 0) {
      Serial.print(F("No activate SPI device: "));
      Serial.println(spi->pinSS());
      AD7190_begin(spi);
      return false;
    }
  }
  
  uint8_t adStatus = AD7190_readStatus(spi);
  if ((adStatus & 0x80) == 0x80) {

#ifdef AD7190_DEBUG_CALLS
  Serial.print(F("AD7190 Status not ready. Status: "));
  Serial.println(adStatus, HEX);
#endif
    
    // AD7190 data not ready
    AD7190_waitMisoGoLow();
  }
  
  uint32_t rawRead = AD7190_readAvg(spi, 1);

  AD7190_rawZero = 0x7FD9F;
  //int32_t temp = rawRead - 0x80000;                        // 
  int32_t t = rawRead - AD7190_rawZero;
  //float weight = t / AD7190_rawFactor;
  int32_t weight = t;
    
#ifdef AD7190_DEBUG_VERBOSE
    Serial.print("ts: ");
    Serial.print(millis());             // Print time stamp
    
    Serial.print(", AD: 0x");
    Serial.print(rawRead, HEX);         // Print Raw ADC value
    
    Serial.print(", status: 0x");
    Serial.print(adStatus, HEX);        //  Print AD7190 Status
    
    Serial.print(", Weight Value: ");
    Serial.print(weight);
    Serial.print("(g)");                //  Print wheight
    Serial.println();
#endif

  Serial.println(weight);

  //AD7190_queueData(spi, weight);

  return true;
}

void AD7190_queueData(SPIClass *spi, float w) {
  
  char ad_data[MAX_MESSAGE_BYTES];
  memset(ad_data, 0, sizeof(ad_data));
  sprintf(ad_data, "{\"%s\", %f, %d}\n", spi->pinSS(), w, millis());
  
  xQueueSend(AD7190_spiQueue, &ad_data, (TickType_t) 100);
}

uint32_t AD7190_readAvg(SPIClass *spi, uint8_t sampleNumber) {

  uint32_t samplesAverage = 0;
  for (uint8_t i = 0; i < sampleNumber; i++) {
    AD7190_waitMisoGoLow();
    uint32_t v = AD7190_getRegisterValue(spi, AD7190_REG_DATA, 3);
    
    uint8_t channel = v & 0x7;   
    samplesAverage += (v >> 4);
  }
  
  samplesAverage = samplesAverage / sampleNumber;
  return samplesAverage ;

}

void spiCommand(SPIClass *spi, byte data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(AD7190_SPI_SETTINGS);
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

void AD7190_begin(SPIClass *spi){
  
  // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  spi->begin();

  // SPI_CLOCK_DIV8 = 250 ns * 2
  // SPI_CLOCK_DIV2 = 62 ns * 2
  // spi->setClockDivider(SPI_CLOCK_DIV8);
  // spi->setDataMode(SPI_MODE3);
  // spi->setBitOrder(MSBFIRST);

  // Set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(spi->pinSS(), OUTPUT); //HSPI SS

  AD7190_reset(spi);                       //  Reset AD7190 communication

  // Check ID value hardcoded in AD7190 registers
  if(AD7190_checkId(spi)){
    Serial.println("AD7190 Init OK!");
    
    AD7190_activeState = true;
    Ad7190SpiErrorCount = 0;
    
  }else{
    Serial.print("Fail init AD7190: ");
    Serial.println(spi->pinSS());
    
    AD7190_activeState = false;
    Ad7190SpiErrorCount = 1;
  }

  AD7190_rawZero = 0;             // TODO: set real zero
  AD7190_rawFactor = 1;           // TODO: set proper scale factor

}

void AD7190_reset(SPIClass *spi){

  //  The serial interface can be reset by writing a series of 1s to the
  //  DIN input. If a Logic 1 is written to the AD7190 DIN line for at
  //  least 40 serial clock cycles, the serial interface is reset. This ensures
  //  that the interface can be reset to a known state if the interface gets
  //  lost due to a software error or some glitch in the system. Reset
  //  returns the interface to the state in which it is expecting a write to
  //  the communications register. This operation resets the contents of
  //  all registers to their power-on values. Following a reset, the user
  //  should allow a period of 500 µs before addressing the serial
  //  interface.

#ifdef AD7190_DEBUG_CALLS
  Serial.println("AD7190 reset");
  delay(10);                              // To be removed
#endif
  
  unsigned char register_word[6];         // At least 40? 5*8? 6*8?
  register_word[0] = 0xFF;
  register_word[1] = 0xFF;
  register_word[2] = 0xFF;
  register_word[3] = 0xFF;
  register_word[4] = 0xFF;
  register_word[5] = 0xFF;
 
  spi->beginTransaction(AD7190_SPI_SETTINGS);
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(register_word, sizeof(register_word));
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();

  delay(500);
}

uint8_t AD7190_readStatus(SPIClass *spi) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println("AD7190_readStatus()");
#endif
  
  uint32_t retValue = AD7190_getRegisterValue(spi, AD7190_REG_STAT, 1);
  uint8_t ad7190Status = (uint8_t)(retValue & 0xFF);

#ifdef AD7190_DEBUG_CALLS

  bool rdy =        !((retValue & 0x80) >> 7);
  bool err =        (retValue & 0x40) >> 6;
  bool noref =      (retValue & 0x20) >> 5;
  bool parity =     (retValue & 0x10) >> 4;
  uint8_t channel = (retValue & 0x07);
  
  Serial.print("AD7190 Status: ");
  Serial.print(retValue, HEX);

  if (rdy){
    Serial.print(" [RDY] ");
  }else{
    Serial.print(" [NOT RDY] ");
  }
  
  if (err){
    Serial.print(" [ERR] ");
  }else{
    Serial.print(" [E_OK] ");
  }

  if (noref){
    Serial.print(" [NO_REF] ");
  }else{
    Serial.print(" [R_OK] ");
  }

  if (parity){
    Serial.print(" [O] ");          //  Odd number of 1
  }else{
    Serial.print(" [E] ");          //  Even number of 1
  }

  Serial.print("Channel: ");
  Serial.println(channel);
  
#endif
  
  return ad7190Status;
  
}

bool AD7190_checkId(SPIClass *spi){

  uint32_t returnValue = AD7190_getRegisterValue(spi, AD7190_REG_ID, 1);

  uint8_t return_id = returnValue & AD7190_ID_MASK;
  if (return_id == ID_AD7190){
    return true;
  }
  return false;
}

float AD7190_readTemperature(SPIClass *spi) {

  float temperature = 0x0;
  uint32_t dataRegReturned = 0x0;

  AD7190_setPolarityRangeBuff(spi, 0, AD7190_CONF_GAIN_1, 1);       // Bipolar mode should be set to read temperature
  AD7190_setChannel(spi, AD7190_CH_TEMP_SENSOR);
  
  dataRegReturned = AD7190_getSingleConversion(spi);

#ifdef AD7190_DEBUG_CALLS
  Serial.print("Temp dataReg: ");
  Serial.println(dataRegReturned);
#endif
  
  temperature = dataRegReturned - 0x800000;
  temperature /= 2815;            // Kelvin Temperature
  temperature -= 273;             //Celsius Temperature
  
#ifdef AD7190_DEBUG_CALLS
  Serial.print("Temp: ");
  Serial.println(temperature);
#endif
  
  return temperature;
}

// Buffer is set to 1
void AD7190_setPolarityRangeBuff(SPIClass *spi, uint8_t polarity, uint8_t range, uint8_t buff) {

#ifdef AD7190_DEBUG_CALLS
    Serial.print("AD7190_rangeSetup. Polarity: ");
    Serial.print(polarity);
    Serial.print(", Range: ");
    Serial.print(range);
    Serial.print(", Buffer: ");
    Serial.println(buff);
#endif

  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  oldRegValue = AD7190_getRegisterValue(spi, AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(AD7190_CONF_GAIN_MASK) | AD7190_CONF_BUF);   // Mask to modify ONLY these 3 registers
   
  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | (buff * AD7190_CONF_BUF);
  AD7190_setRegisterValue(spi, AD7190_REG_CONF, newRegValue, 3);
}

void AD7190_setChannel(SPIClass *spi, unsigned short channel){
  
#ifdef AD7190_DEBUG_CALLS
    Serial.print("AD7190_setChannel, ");
    Serial.println(channel);
#endif

  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  oldRegValue = AD7190_getRegisterValue(spi, AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
  
  newRegValue = oldRegValue | AD7190_CONF_CHAN(channel);
  AD7190_setRegisterValue(spi, AD7190_REG_CONF, newRegValue, 3);

  // Whait MISO to go LOW?
  AD7190_waitMisoGoLow();
  
}

uint32_t AD7190_getSingleConversion(SPIClass *spi) {
  
  #ifdef AD7190_DEBUG_CALLS
    Serial.println("AD7190_singleConversion");
  #endif
  
  uint32_t command = 0x0;
  uint32_t regData = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_1023);
  
  AD7190_setRegisterValue(spi, AD7190_REG_MODE, command, 3);
  AD7190_waitMisoGoLow();
  
  regData = AD7190_getRegisterValue(spi, AD7190_REG_DATA, 3);
  return regData;
}

bool AD7190_waitMisoGoLow(void) {
  
  //  RDY only goes low when a valid conversion is available
  
  //  The DOUT/ RDY pin functions as a data ready signal also, the
  //  line going low when a new data-word is available in the output
  //  register. It is reset high when a read operation from the data
  //  register is complete. It also goes high prior to the updating of the
  //  data register to indicate when not to read from the device, to
  //  ensure that a data read is not attempted while the register is being
  //  updated. 

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190_waitMisoGoLow()"));
#endif

  uint32_t timeOutMilis = AD7190_DOUT_TIMEOUT;
  uint8_t rdyPin = digitalRead(SPI_MISO);
  
  while (rdyPin && timeOutMilis) {
    delay(1);
    rdyPin = digitalRead(SPI_MISO);
    timeOutMilis--;
  }

#ifdef AD7190_DEBUG_CALLS
  if (!timeOutMilis) {
    Serial.println(F("Timeout: AD7190_waitMisoGoLow()"));
  }
  if (!rdyPin) {
    Serial.println(F("rdyPin ZERO: AD7190_waitMisoGoLow()"));
  }
#endif

  if (!rdyPin) {
    return false;
  }
  return true;
}

#ifdef AD7190_DEBUG_CALLS
char*  AD7190_getAddressDebugString(uint8_t a){

  bool writeEnable =    (a & 0x80) >> 7;
  bool writeOperation = !((a & 0x40) >> 6);
  uint8_t regAdd =      (a & 0x38) >> 3;
  bool creadRead =      (a & 0x04) >> 2;

  char we[3];
  if (writeEnable){
    strcpy(we,"WE");
  }else{
    strcpy(we,"  ");
  }
  char w[2];
  if (writeOperation){
    strcpy(w,"W");
  }else{
    strcpy(w,"R");
  }
  char c[3];
  if (creadRead){
    strcpy(c,"CR");
  }else{
    strcpy(c,"  ");
  }

  char regAdd_char[20];
  strcpy(regAdd_char,"AD7190_UNKNOWN");
    
  switch(regAdd) {
    case 0:
      if (writeOperation){
        strcpy(regAdd_char,"AD7190_REG_COMM");
      }else{
        strcpy(regAdd_char,"AD7190_REG_STAT");
      }
      break;
      
    case 1:
      strcpy(regAdd_char,"AD7190_REG_MODE");
      break;
      
    case 2:
      strcpy(regAdd_char,"AD7190_REG_CONF");
      break;
      
    case 3:
      if (!writeOperation){
        strcpy(regAdd_char,"AD7190_REG_DATA");
      }
      break;
      
    case 4:
      if (!writeOperation){
        strcpy(regAdd_char,"AD7190_REG_ID");
      }
      break;
      
    case 5:
      strcpy(regAdd_char,"AD7190_REG_GPOCON");
      break;
      
    case 6:
      strcpy(regAdd_char,"AD7190_REG_OFFSET");
      break;
      
    case 7:
      strcpy(regAdd_char,"AD7190_REG_FULLSCALE");
      break;
      
    default:
      strcpy(regAdd_char,"AD7190_UNKNOWN");
    } 
  
  static char bufferDebugAddress[40];
  char* formato="[%s][%s][%s] (%d) %s";
  sprintf(bufferDebugAddress, formato, we, w, c, regAdd, regAdd_char);

  return bufferDebugAddress;  
}
#endif

uint32_t AD7190_getRegisterValue(SPIClass *spi, byte registerAddress, uint8_t bytesNumber) {

  byte inByte = 0;           // incoming byte from the SPI
  uint32_t result = 0;   // result to return

  uint8_t address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);

#ifdef AD7190_DEBUG_CALLS
  uint8_t responseBytes = bytesNumber;
  
  Serial.print("AD7190_GetRegisterValue Add: 0x");
  Serial.print(address, HEX);
  Serial.print(" ");
  Serial.print(AD7190_getAddressDebugString(address));
  Serial.print(", ResponseSize: ");
  Serial.println(responseBytes);
#endif

  spi->beginTransaction(AD7190_SPI_SETTINGS);
  digitalWrite(spi->pinSS(), LOW); // Pull SS slow to prep other end for transfer

  spi->transfer(address);          // send the device the register you want to read:

  result = spi->transfer(0x00);    // Send a value of 0 to read the first byte returned:

  bytesNumber--;                   // decrement the number of bytes left to read:
  while (bytesNumber > 0) {        // if you still have another byte to read:
    result = result << 8;          // shift the first byte left,
    inByte = spi->transfer(0x00);  // then get the second byte:
    result = result | inByte;      // combine the byte you just got with the previous ones
    bytesNumber--;                 // decrement the number of bytes left to read
  }
  
  digitalWrite(spi->pinSS(), HIGH); // Pull SS pin HIGH to signify end of data transfer
  spi->endTransaction();

#ifdef AD7190_DEBUG_CALLS                                   // This block is printing on Serial:
  uint8_t b = 0;                                      // a string like: AD7190_Response:  [2|0xHH]  [1|0xHH]  [0|0xHH]
  Serial.print("AD7190_Response: ");                  // this is only for debuging
  for (int8_t i = responseBytes-1; i >= 0; i--) {    
      b = (result >> (8 * i)) & 0xFF;
      Serial.printf(" [%d|0x%02x] ", i, b);
  }
  Serial.println();
#endif

  return (result);
}

void AD7190_setRegisterValue(SPIClass *spi, unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber) {
  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&registerValue;
  unsigned char bytesNr         = bytesNumber;

  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);

  while (bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }

  #ifdef AD7190_DEBUG_CALLS
    Serial.print("AD7190_setRegisterValue Add: 0x");
    Serial.print(writeCommand[0], HEX);
    Serial.print(" ");
    Serial.print(AD7190_getAddressDebugString(writeCommand[0]));
   
    Serial.print(", Payload:");
    for (uint8_t i = 1; i <= bytesNumber; i ++) {
        Serial.printf(" [%d|0x%02x] ", i, writeCommand[i]);
    }
    Serial.print(" Size: ");
    Serial.println(bytesNumber);
  #endif

  spi->beginTransaction(AD7190_SPI_SETTINGS);       // Should be after pinSS pull down?
  digitalWrite(spi->pinSS(), LOW);                  // Pull SS slow to prep other end for transfer
  delayMicroseconds(1);                             // TO DO: Can this delay be removed?

  spi->transfer(writeCommand, bytesNumber + 1);

  digitalWrite(spi->pinSS(), HIGH);                   // Pull SS high to signify end of data transfer
  spi->endTransaction();
  
}
