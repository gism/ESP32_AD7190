#include "wifiTasks.h"

WiFiManager wm;                   // global WifiManager instance
WiFiClient wifiClient;

const IPAddress serverIP(192, 168, 0, 14);    // Server IP
uint16_t serverPort = 8000;                   // Server Port

uint32_t counter = 0;
char bufferServer[60];

void configureWifi(){
  
  wm.setDebugOutput(false);
    
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  

  // wm.resetSettings(); // wipe settings

  //set static ip
  // wm.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0)); // set static ip,gw,sn
  // wm.setShowStaticFields(true); // force show static ip fields
  // wm.setShowDnsFields(true);    // force show dns field always

  // wm.setConnectTimeout(20); // how long to try to connect for before continuing
  // wm.setConfigPortalTimeout(300); // auto close configportal after n seconds
  // wm.setCaptivePortalEnable(false); // disable captive portal redirection
  // wm.setAPClientCheck(true); // avoid timeout if client connected to softap

  // wifi scan settings
  // wm.setRemoveDuplicateAPs(false); // do not remove duplicate ap names (true)
  // wm.setMinimumSignalQuality(20);  // set min RSSI (percentage) to show in scans, null = 8%
  // wm.setShowInfoErase(false);      // do not show erase button on info page
  // wm.setScanDispPerc(true);       // show RSSI as percentage not graph icons
  
  // wm.setBreakAfterConfig(true);   // always exit configportal even if wifi save fails
  
  bool res;
  res = wm.autoConnect(WIFI_AP_NAME);           // password protected ap

  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    //ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println(F("Wifi: Connected (OK)"));
  }

  printWifiDebugInfo();
  
}

void printWifiDebugInfo(){

  printf("Up time: %dmin %dsec\n", (millis() / 1000 / 60), ((millis() / 1000) % 60));
  printf("Connected to SSID: %s\n", WiFi.SSID());
  printf("ESP32 IP: %s\n", WiFi.localIP().toString());
  printf("Gateway IP: %s\n", WiFi.gatewayIP().toString());
  printf("Subnet Mask: %s\n", WiFi.subnetMask().toString());
  printf("DNS: %s\n", WiFi.dnsIP().toString());
  printf("Host Namek: %s\n", WiFi.getHostname());
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
}


bool connectServer() {
  
  Serial.print(F("Connecting socket server: IP: "));
  Serial.print(serverIP);
  Serial.print(", Port=");
  Serial.println(serverPort);
 
  wifiClient.setTimeout(5);  // seconds
  if (wifiClient.connect(serverIP, serverPort)) {
    Serial.println(F("Connect socket success"));
    return true;
  } else {
    Serial.println(F("Connect socket failed."));
  }
  return false;
}


bool serverIsConnected(){
  return wifiClient.connected();
}

bool sendInfoServer(){
    
    sprintf(bufferServer, "Counter from ESP: %d ticks\n", counter);
    
    int n = strlen(bufferServer);
    //int ret = wifiClient.write(bufferServer, n);
    int ret = wifiClient.print(bufferServer);

    Serial.print("Send len=");
    Serial.print(ret);
    Serial.print(" - ");
    Serial.print(bufferServer);

    counter = counter + 1;

    while (wifiClient.available())
    {
      String line = wifiClient.readStringUntil('\n');
      Serial.print("read data：");
      Serial.println(line);
    }

    return true;
}


bool streamAd7190DataMessageToServer(char* messageBlock) {


    if (!messageBlock) {
    #ifdef MAIN_DEBUG_VERBOSE
      Serial.println(F("Error: streamAd7190DataMessageToServer, messageBlock is NULL"));
    #endif
      return false;
    }
    
    int n = strlen(messageBlock);
    //int ret = wifiClient.write(messageBlock, n);
    int ret = wifiClient.print(messageBlock);

    //#ifdef MAIN_DEBUG_VERBOSE
    Serial.print("Send len=");
    Serial.print(ret);
    Serial.print(" - ");
    Serial.print(messageBlock);
    //#endif

    while (wifiClient.available())
    {
      String line = wifiClient.readStringUntil('\n');
      Serial.print("read data：");
      Serial.println(line);
    }

    return true;
}
