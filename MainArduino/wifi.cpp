#include "wifi.h"

#include "ESP8266.h"

#define SSID        "WLAN-2G"
#define PASSWORD    "14935251"
#define HOST_NAME   "192.168.56.119"
#define HOST_PORT   (11411)

ESP8266 wifi(Serial2, 115200);


void WIFI::setup(){
    Serial.print("setup begin\r\n");
    
    Serial.print("FW Version:");
    Serial.println(wifi.getVersion().c_str());
      
    if (wifi.setOprToStationSoftAP()) {
        Serial.print("to station + softap ok\r\n");
    } else {
        Serial.print("to station + softap err\r\n");
    }
 
    if (wifi.joinAP(SSID, PASSWORD)) {
        Serial.print("Join AP success\r\n");
        Serial.print("IP:");
        Serial.println( wifi.getLocalIP().c_str());       
    } else {
        Serial.print("Join AP failure\r\n");
    }
    
    if (wifi.disableMUX()) {
        Serial.print("single ok\r\n");
    } else {
        Serial.print("single err\r\n");
    }

    if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
        Serial.print("create tcp ok\r\n");
    } else {
        Serial.print("create tcp err\r\n");
    }
    
    Serial.print("setup end\r\n");
 }


 int WIFI::read () {
    if (_len > 0) {
      int c = _buffer[_index++];
      _len--;
      return c;
    } else {
      _len = wifi.recv(_buffer, sizeof(_buffer), 100);
      _index = 0;
      return -1;
    }
 }
 void WIFI::send (uint8_t* data, int length) {
    if (wifi.send(data, length) == false) {
        if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
            Serial.print("create tcp ok\r\n");
        } else {
            Serial.print("create tcp err\r\n");
        }  
    };
 }
 
 void WIFI::loop () {
 /*   uint8_t buffer[128] = {0};
    
    if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
        Serial.print("create tcp ok\r\n");
    } else {
        Serial.print("create tcp err\r\n");
    }
    
    char *hello = "Hello, this is client!";
    wifi.send((const uint8_t*)hello, strlen(hello));
    
    uint32_t len = wifi.recv(buffer, sizeof(buffer), 10000);
    if (len > 0) {
        Serial.print("Received:[");
        for(uint32_t i = 0; i < len; i++) {
            Serial.print((char)buffer[i]);
        }
        Serial.print("]\r\n");
    }
    
    if (wifi.releaseTCP()) {
        Serial.print("release tcp ok\r\n");
    } else {
        Serial.print("release tcp err\r\n");
    }
    delay(5000);    
    */  
 }
