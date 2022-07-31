#include "AD7190.h"

AD7190::AD7190(SPIClass * spiClass, uint8_t pin_rdy) {
  
#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190::AD7190(SPIClass * spiClass"));
#endif

  this->spiClass = spiClass;
  this->pin_rdy = pin_rdy;

  this->activeState = true;
  this->spiErrorCount = 0;

}

AD7190::AD7190(SPIClass * spiClass, uint8_t pin_rdy, char* deviceName) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190::AD7190(SPIClass * spiClass, char* deviceName"));
#endif

  this->spiClass = spiClass;
  this->pin_rdy = pin_rdy;
  
  size_t n = strlen(deviceName);
  if (n > 15) n = 15; // max len of name is 15
  
  memset(this->deviceName, 0, sizeof(this->deviceName));
  memcpy(this->deviceName, deviceName, n);

#ifdef AD7190_DEBUG_VERBOSE
  Serial.print(F("AD7190 deviceName: "));
  Serial.println(this->deviceName);
#endif

  this->activeState = true;
  this->spiErrorCount = 0;
}

AD7190::~AD7190(){
}

char* AD7190::getDeviceName() {
  return this->deviceName;
}

bool AD7190::begin(){

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.begin()"));
#endif
  
  spiClass->begin();

  // Set up slave select pins as outputs as the Arduino API
  // doesn't handle automatically pulling SS low
  pinMode(spiClass->pinSS(), OUTPUT);

  // Reset AD7190 communication
  reset();
  
  if(checkId()){    // Check ID value hardcoded in AD7190 registers

    #ifdef AD7190_DEBUG_VERBOSE
      Serial.println(F("AD7190 Init OK"));
    #endif
        
    this->activeState = true;
    this->spiErrorCount = 0;
    
  }else{

    #ifdef AD7190_DEBUG_VERBOSE
      Serial.print(F("AD7190 Init FAIL. CS:"));
      Serial.println(spiClass->pinSS());
    #endif
        
    this->activeState = false;
    this->spiErrorCount = this->spiErrorCount + 1;
  }

  return this->activeState;
}

void AD7190::reset(){

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
  Serial.println(F("AD7190.reset()"));
#endif
  
  unsigned char register_word[6];         // At least 40? 5*8? 6*8?
  register_word[0] = 0xFF;                // can be reduced to only 5 bytes
  register_word[1] = 0xFF;
  register_word[2] = 0xFF;
  register_word[3] = 0xFF;
  register_word[4] = 0xFF;
  register_word[5] = 0xFF;
 
  spiClass->beginTransaction(AD7190_SPI_SETTINGS);
  digitalWrite(spiClass->pinSS(), LOW);                         //  Pull CS low to prep other end for transfer
  
  spiClass->transfer(register_word, sizeof(register_word));
  
  digitalWrite(spiClass->pinSS(), HIGH);                        //  Pull CS high to signify end of data transfer
  spiClass->endTransaction();

  //  Following a reset, the user should allow a period of 500 µs 
  //  before addressing the serial interface. 
  delay(500);
}

bool AD7190::checkId(){
  
#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.checkId()"));
#endif

  uint32_t returnValue = getRegisterValue(AD7190_REG_ID, 1);

  uint8_t return_id = returnValue & AD7190_ID_MASK;
  if (return_id == ID_AD7190){
    return true;
  }
  return false;
}


float AD7190::getTemperature() {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.getTemperature()"));
#endif

  setConfigurationRegister(AD7190_CH_TEMP_SENSOR, AD7190_BUFF_ACTIVE, AD7190_BIPOLAR, AD7190_CONF_GAIN_1);
 
  uint32_t command = 0x0;
  uint32_t regDataReturned = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_1023);
  setRegisterValue(AD7190_REG_MODE, command, 3);

  waitMisoGoLow();        // Whait MISO/RDY to go LOW
  
  regDataReturned = getRegisterValue(AD7190_REG_DATA, 3);


#ifdef AD7190_DEBUG_VERBOSE
  Serial.print(F("Temperature Reg: "));
  Serial.print(regDataReturned, HEX);
#endif

  float temperature = 0x0;
  temperature = regDataReturned - 0x800000;
  temperature /= 2815;            // Kelvin Temperature
  temperature -= 273;             //Celsius Temperature
  
#ifdef AD7190_DEBUG_VERBOSE
  Serial.print(F(" --> AD7190 Temperature: "));
  Serial.println(temperature);
#endif
  
  return temperature;
}

void AD7190::setConfigurationRegister(uint8_t channel, uint8_t buff, uint8_t polarity, uint8_t range) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.setConfigurationRegister(...)"));
#endif

  uint32_t oldRegValue = 0x0;
  uint32_t newRegValue = 0x0;

  // Get current register value of Configuration Register:
  oldRegValue = getRegisterValue(AD7190_REG_CONF, 3);
  
  // Mask to modify ONLY these 3 bits
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(AD7190_CONF_GAIN_MASK) | AD7190_CONF_BUF | AD7190_CONF_CHAN(0xFF));

  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | (buff * AD7190_CONF_BUF | AD7190_CONF_CHAN(channel));
  setRegisterValue(AD7190_REG_CONF, newRegValue, 3);

  waitMisoGoLow();        // Whait MISO/RDY to go LOW
 
}


uint8_t AD7190::getStatusRegister() {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.readStatusRegister()"));
#endif
  
  uint32_t retValue = getRegisterValue(AD7190_REG_STAT, 1);
  uint8_t ad7190Status = (uint8_t)(retValue & 0xFF);

#ifdef AD7190_DEBUG_VERBOSE

  bool rdy =        !((retValue & 0x80) >> 7);
  bool err =         (retValue & 0x40) >> 6;
  bool noref =       (retValue & 0x20) >> 5;
  bool parity =      (retValue & 0x10) >> 4;
  uint8_t channel =  (retValue & 0x07);
  
  Serial.print(F("AD7190 Status: "));
  Serial.print(retValue, HEX);

  if (rdy){
    Serial.print(F(" [RDY] "));
  }else{
    Serial.print(F(" [NOT RDY] "));
  }
  
  if (err){
    Serial.print(F(" [ERR] "));
  }else{
    Serial.print(F(" [E_OK] "));
  }

  if (noref){
    Serial.print(F(" [NO_REF] "));
  }else{
    Serial.print(F(" [R_OK] "));
  }

  if (parity){
    Serial.print(F(" [Od] "));          //  Odd number of 1
  }else{
    Serial.print(F(" [Ev] "));          //  Even number of 1
  }

  Serial.print(F("Channel: "));
  Serial.println(channel);
  
#endif
  
  return ad7190Status;
  
}

uint32_t AD7190::getDataRegisterAvg(uint8_t sampleNumber) {

  uint32_t samplesAverage = 0;
  for (uint8_t i = 0; i < sampleNumber; i++) {
    waitMisoGoLow();                                                  //  TODO: Check if this can be removed
    uint32_t v = getRegisterValue(AD7190_REG_DATA, 3);
    
    uint8_t channel = v & 0x7;   
    samplesAverage += (v >> 4);
  }
  
  samplesAverage = samplesAverage / sampleNumber;
  return samplesAverage ;

}


uint32_t AD7190::getDataRegister(uint8_t sampleNumber) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.getData()"));
#endif
  
  if (!this->activeState) {
    this->spiErrorCount = this->spiErrorCount + 1;
    
    if (this->spiErrorCount % 100 == 0) {

      #ifdef AD7190_DEBUG_VERBOSE
        Serial.print(F("No activate SPI device: "));
        Serial.print(spiClass->pinSS());
      #endif

      begin();
      return 0;
    }
  }
  
  uint8_t adStatus = getStatusRegister();
  if ((adStatus & 0x80) == 0x80) {

  #ifdef AD7190_DEBUG_VERBOSE
    Serial.print(F("AD7190 Status not ready. Status: "));
    Serial.println(adStatus, HEX);
  #endif
    
    waitMisoGoLow();                    // AD7190 data not ready.
  }
  
  uint32_t rawRead = getDataRegisterAvg(sampleNumber);
    
#ifdef AD7190_DEBUG_VERBOSE  
    Serial.print(F("AD7190.getDataRegister() data: "));
    Serial.print(rawRead, HEX);                             // Print Raw ADC value
#endif

  return rawRead;
  
}

bool AD7190::waitMisoGoLow(void) {
  
  //  RDY only goes low when a valid conversion is available
  
  //  The DOUT/ RDY pin functions as a data ready signal also, the
  //  line going low when a new data-word is available in the output
  //  register. It is reset high when a read operation from the data
  //  register is complete. It also goes high prior to the updating of the
  //  data register to indicate when not to read from the device, to
  //  ensure that a data read is not attempted while the register is being
  //  updated. 

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.waitMisoGoLow()"));
#endif

  uint32_t timeOutMilis = AD7190_DOUT_TIMEOUT;
  uint8_t rdyPin = digitalRead(this->pin_rdy);
  
  while (rdyPin && timeOutMilis) {
    delay(1);
    rdyPin = digitalRead(this->pin_rdy);
    timeOutMilis--;
  }

#ifdef AD7190_DEBUG_VERBOSE
  if (!timeOutMilis) {
    Serial.println(F("AD7190.waitMisoGoLow() TIMEOUT!"));
  }
  if (!rdyPin) {
    Serial.println(F("AD7190.waitMisoGoLow() OK, RDY pin low"));
  }
#endif

  if (!rdyPin) {
    return false;
  }
  return true;
}

uint32_t AD7190::getRegisterValue(byte registerAddress, uint8_t bytesNumber) {
  
#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.getRegisterValue()"));
#endif

  byte inByte = 0;           // incoming byte from the SPI
  uint32_t result = 0;       // result to return

  uint8_t address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);

#ifdef AD7190_DEBUG_VERBOSE
  Serial.print(F("AD71900 GetRegisterValue Add: 0x"));
  Serial.print(address, HEX);
  Serial.print(" ");
  Serial.print(getAddressDebugString(address));
  Serial.print(F(", ResponseSize: "));
  Serial.println(bytesNumber);

  uint8_t responseBytes = bytesNumber;              // for later
#endif

  spiClass->beginTransaction(AD7190_SPI_SETTINGS);
  digitalWrite(spiClass->pinSS(), LOW);             // Pull SS slow to prep other end for transfer

  spiClass->transfer(address);                      // send the device the register you want to read:

  result = spiClass->transfer(0x00);                // Send a value of 0 to read the first byte returned:

  bytesNumber--;                                    // decrement the number of bytes left to read:
  while (bytesNumber > 0) {                         // if you still have another byte to read:
    result = result << 8;                           // shift the first byte left,
    inByte = spiClass->transfer(0x00);              // then get the second byte:
    result = result | inByte;                       // combine the byte you just got with the previous ones
    bytesNumber--;                                  // decrement the number of bytes left to read
  }
  
  digitalWrite(spiClass->pinSS(), HIGH);            // Pull SS pin HIGH to signify end of data transfer
  spiClass->endTransaction();

#ifdef AD7190_DEBUG_VERBOSE                                   // This block is printing on Serial:
  uint8_t b = 0;                                              // a string like: AD7190_Response:  [2|0xHH]  [1|0xHH]  [0|0xHH]
  Serial.print(F("AD7190 Response:"));                          // this is only for debuging
  for (int8_t i = responseBytes-1; i >= 0; i--) {    
      b = (result >> (8 * i)) & 0xFF;
      Serial.printf(" [%d|0x%02x] ", i, b);
  }
  Serial.println();
#endif

  return (result);
}

void AD7190::setRegisterValue(unsigned char registerAddress, uint32_t registerValue, unsigned char bytesNumber) {

#ifdef AD7190_DEBUG_CALLS
  Serial.println(F("AD7190.setRegisterValue()"));
#endif
  
  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&registerValue;
  unsigned char bytesNr         = bytesNumber;

  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);

  while (bytesNr > 0) {
    writeCommand[bytesNr] = *dataPointer;
    dataPointer ++;
    bytesNr --;
  }

  #ifdef AD7190_DEBUG_VERBOSE
    Serial.print(F("AD7190 setRegisterValue Add: 0x"));
    Serial.print(writeCommand[0], HEX);
    Serial.print(" ");
    Serial.print(getAddressDebugString(writeCommand[0]));
   
    Serial.print(F(", Payload:"));
    for (uint8_t i = 1; i <= bytesNumber; i ++) {
        Serial.printf(" [%d|0x%02x] ", i, writeCommand[i]);
    }
    Serial.print(F(" Size: "));
    Serial.println(bytesNumber);
  #endif

  spiClass->beginTransaction(AD7190_SPI_SETTINGS);
  
  digitalWrite(spiClass->pinSS(), LOW);                   // Pull SS slow to prep other end for transfer
  spiClass->transfer(writeCommand, bytesNumber + 1);

  digitalWrite(spiClass->pinSS(), HIGH);                   // Pull SS high to signify end of data transfer
  spiClass->endTransaction();
}

#ifdef AD7190_DEBUG_VERBOSE
char*  AD7190::getAddressDebugString(uint8_t a){

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
