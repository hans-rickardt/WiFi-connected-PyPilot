#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
//#include <Bounce2.h>                     // https://github.com/thomasfredericks/Bounce2
#include "HARestAPI.h"
//#include "HARestAPI.h"
//#include <WiFiClientSecureBearSSL.h>
//#include "secret.h";

#define BUTTON_PIN 0 // Flash button on NodemMCU

//Comment/Uncomment if HA is using SSL
//WiFiClientSecure sclient;
//HARestAPI ha(sclient);

//Comment/Uncomment if HA is not using SSL
WiFiClient client;
HARestAPI ha(client);

#ifndef SECRET
const char* ssid = "hans6";
const char* password = "kanelbulle20";
const char* ha_ip = "10.0.1.109";
uint16_t ha_port = 8123; // Could be 443 is using SSL
const char* ha_pwd = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiIxMmEyNGI1NzgwMzU0ZTNkYjg1YTJlZWE1MzE4YzY5NyIsImlhdCI6MTcwNTUwNTI0MywiZXhwIjoyMDIwODY1MjQzfQ.U-lnRSnKsGz3kyiYSC91kT5IDLvFp8f4Gp9pXXXGcV8";  //long-lived password. On HA, Profile > Long-Lived Access Tokens > Create Token
//String fingerprint = "35 85 74 EF 67 35 A7 CE 40 69 50 F3 C0 F6 80 CF 80 3B 2E 19";
#endif

//Bounce debouncer = Bounce(); 

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  ha.setHAServer(ha_ip, ha_port);
  ha.setDebugMode(0);
  ha.setHAPassword(ha_pwd);
  //ha.setFingerPrint(fingerprint); // Only used if HA is running SSL
  pinMode(BUTTON_PIN,INPUT_PULLUP);
///  debouncer.attach(BUTTON_PIN);
//  debouncer.interval(50);
}

void loop() {

  String t;
 // debouncer.update();
 // if ( debouncer.fell() ) {
    // All the commands below do the same thing
    // Home Assistant
    // URL: "/api/services/light/turn_on" (HA -> Developer Tools -> Services -> Service)
    // Component: "light.bedroom_light" (HA -> Developer Tools -> Services -> Entity)
  
    
    //// 1. Send custom DATA to HA
    //ha.sendCustomHAData("/api/services/light/turn_on", "{\"entity_id\":\"light.bedroom_light\"}");
  
    //// 2. Set component and send light to HA // Will set tmpURL to "/api/services/light/turn_on"
    //ha.setComponent("light.bedroom_light");
    //ha.sendHALight(true);
  
    //// 3. Set light on/off as bool and component is 2nd input, Send Light to HA // Will set tmpURL to "/api/services/light/turn_on"
    //ha.sendHALight(true, "light.bedroom_light");
  
    //// 4. Set component and URL is 2nd input, Send custom URL to HA
    //ha.setComponent("light.bedroom_light");
    //ha.sendHAURL("/api/services/light/turn_on");
  
    //// 5. Set URL and Send component to HA
    //ha.setURL("/api/services/light/turn_on");
    //ha.sendHAComponent("light.bedroom_light");
  
    //// 6. Set URL 1st argument, component 2nd, send to HA
    //ha.sendHAComponent("/api/services/light/turn_on", "light.bedroom_light");
  
    //// 7. Set URL , Set component, send to HA
    //ha.setURL("/api/services/light/turn_on");
    //ha.setComponent("light.bedroom_light");
    //ha.sendHA();
    
    // 8. Set URL and Send area to HA
    //ha.setURL("/api/services/light/turn_on");
    //ha.sendHAArea("bedroom");
  
    // Instead of Turn on light, lets toggle it
//    ha.setURL("/api/services/light/toggle");
    //ha.setURL("/api/services/switch/turn_on");

    //ha.sendHAComponent("switch.lampa_jens");
 // }

//delay(10000);
  //  ha.setURL("/api/services/switch/turn_off");

   // ha.sendHAComponent("switch.lampa_jens");
   // delay(10000);


 t=ha.sendGetHA("/api/states/switch.lampa_jens");
 Serial.println("st result");
 Serial.println(t);
 Serial.println("en result");
 
 delay(10000);
}
