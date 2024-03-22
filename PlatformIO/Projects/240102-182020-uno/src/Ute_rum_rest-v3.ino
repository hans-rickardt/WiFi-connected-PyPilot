/*
  This a simple example of the aREST Library for Arduino (Uno/Mega/Due/Teensy)
  using the WiFi library (for example to be used with the Arduino WiFi shield).
  See the README file for more details.

  Written in 2014 by Marco Schwartz under a GPL license.
*/

//#include <SPI.h>
//#include <WiFi.h>
#include <WiFiNINA.h>
#include <aREST.h>
#include <SparkFunHTU21D.h>
#include <Adafruit_MotorShield.h>
//#include <Wire.h>

HTU21D myHumidity;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myFan = AFMS.getMotor(1);
Adafruit_DCMotor *myW1 = AFMS.getMotor(2);
Adafruit_DCMotor *myW2 = AFMS.getMotor(3);
Adafruit_DCMotor *myW3 = AFMS.getMotor(4);
char ssid[] = "hans6"; //  your network SSID (name)
char pass[] = "kanelbulle20";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Create aREST instance
aREST rest = aREST();

// Initialize the WiFi server library
WiFiServer server(80);

// Variables to be exposed to the API
float uterum_temperature;
float uterum_humidity;
int fan;
int w1pos = 9;
int w2pos = 9;
int w3pos = 9;

// Declare functions to be exposed to the API
int ledControl(String command);

void setup() {

  // Start Serial
  Serial.begin(115200);
  Serial.println("Project Ute_rum_rest-v3");
  pinMode(LED_BUILTIN, OUTPUT);
  myHumidity.begin();

  AFMS.begin();
  //    myFan->setSpeed(150);
  // myFan->run(FORWARD);
  // turn on motor
  // myFan->run(RELEASE);

  // Init variables and expose them to REST API
  uterum_temperature = 24;
  uterum_humidity = 40;
  rest.variable("u_temp", &uterum_temperature);
  rest.variable("u_humi", &uterum_humidity);
  rest.variable("fan", &fan);
  rest.function("led", ledControl);
  // Give name and ID to device (ID should be 6 characters long)

  rest.set_id("000001");
  rest.set_name("Ute_Rum");

  // Function to be exposed
  rest.function("led", ledControl);
  rest.function("fan", fanControl);
  rest.function("wind", windowControl);
  rest.variable("w1pos", &w1pos);
  rest.variable("w2pos", &w2pos);
  rest.variable("w3pos", &w3pos);



  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if ( fv != "1.3.0" )
    Serial.println("Please upgrade the firmware");
  Serial.println(fv);

  // Attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.setHostname("ute_rum");
    status = WiFi.begin(ssid, pass);

    // Wait 10 seconds for connection
    delay(10000);
   
  }

  // Start the server
  server.begin();
  // Print out the status
  printWifiStatus();
    windowControl("w1,0");
    windowControl("w2,0");
    windowControl("w3,0");

}


void loop() {
  static int cnt = 100;
  int W_ST;
  String str;
  String str2;
  int pingResult;
  // listen for incoming clients
  
  WiFiClient client = server.available();
  uterum_temperature = myHumidity.readTemperature();
  uterum_humidity = myHumidity.readHumidity();

  

  cnt--;
  if ( cnt == 0 ) {

    cnt = 5000;
    
//Serial.println(uterum_temperature);
    if( uterum_temperature > 33 )
    { 
      uterum_temperature =33;
    }
    
    if ( uterum_temperature > 24 ) {
    
      str2 = String(uterum_temperature - 24, DEC);
      str = String("w1," + str2);
      //windowControl("w1,7");
      windowControl(str);
      //Serial.println(uterum_temperature);
    }
    
    if ( uterum_temperature > 25 )
    {
      fanControl("0");
      str2 = String(uterum_temperature - 25, DEC);
      str = String("w2," + str2);
      //windowControl("w2,7");
      windowControl(str);
      //str = String("w3," + str2);
      //windowControl(str);
    }
    
 if ( uterum_temperature > 26 )
    {
      fanControl("0");
      str2 = String(uterum_temperature - 26, DEC);
      //str = String("w2," + str2);
      //windowControl("w2,7");
      //windowControl(str);
      str = String("w3," + str2);
      windowControl(str);
    }
    if ( uterum_temperature <24 )
    {
      windowControl("w1,0");
      windowControl("w2,0");
      windowControl("w3,0");
    }

    if ( uterum_temperature > 24)
    {
      fanControl("255");
     }
     else 
     {
      fanControl("0");
     }

  // Test WiFi
  pingResult = WiFi.ping("10.0.1.200");

  if (pingResult >= 0) {
   // Serial.print("SUCCESS! RTT = ");
   // Serial.print(pingResult);
   // Serial.println(" ms");
  } else {
    Serial.print("FAILED! Error code: ");
    Serial.println(pingResult);
    softReset();
  }
 // WiFi
 }

  
  rest.handle(client);
  
delay(1);
}

// Custom function accessible by the API
int ledControl(String command) {

  // Get state from command
  int state = command.toInt();

  digitalWrite(6, state);
  return 1;
}

int fanControl(String command) {
  unsigned char i;
  // Get state from command
  int state = command.toInt();
  Serial.println("fan");
  if ( fan != state )
  { 
  fan = state;
  Serial.println(state);
  //  digitalWrite(2,state);
  if ( fan == 0 )
  {
    myFan->run(RELEASE);
  } else
  {
    myFan->run(FORWARD);
    for (i = 0; i < fan; i++) {
      myFan->setSpeed(i);
      delay(20);
    }
  }
  }
  return 1;
}



/* http://10.0.1.119/wind?params=w1,9 */
int windowControl(String command) {
  unsigned char i;

  // Get state from command
  Serial.println(command);
  int wi = command[1] - 0x30;
  int pos = command[3] - 0x30;
  int diff = 0;
  int delta = 0;
  Serial.println("wind");

  Serial.println(wi);
  Serial.println(pos);

// max Pos 5 levels
  if ( pos > 5 ){
    pos=5;
  }

  switch (wi) {
    case 1:
  //    Serial.println("case1");
      myW1->setSpeed(255);
      diff = w1pos - pos;

      if ( diff > 0 )
      {
        myW1->run(FORWARD);
        delay(3000 * (diff));

      }
      else if ( diff < 0)
      {
        myW1->run(BACKWARD);
        delay(3000 * (0 - diff));
      }
      myW1->run(RELEASE);
      w1pos = pos;
      break;

    case 2:
    //  Serial.println("case2");
      myW2->setSpeed(255);
      diff = w2pos - pos;

      if ( diff > 0 )
      {
        myW2->run(FORWARD);
        delay(3000 * (diff));

      }
      else if ( diff < 0)
      {
        myW2->run(BACKWARD);
        delay(3000 * (0 - diff));
      }
      myW2->run(RELEASE);
      w2pos = pos;
      break;

    case 3:
      //Serial.println("case3");
      myW3->setSpeed(255);
      diff = w3pos - pos;

      if ( diff > 0 )
      {
        myW3->run(FORWARD);
        delay(3000 * (diff));

      }
      else if ( diff < 0)
      {
        myW3->run(BACKWARD);
        delay(3000 * (0 - diff));
      }
      myW3->run(RELEASE);
      w3pos = pos;
      break;

  }


  /*  if ( fan == 0 )
    {
       myFan->run(RELEASE);
    }else
    {
    myFan->run(FORWARD);
     for (i=0; i<fan; i++) {
      myFan->setSpeed(i);
      delay(20);
      }
    }
  */
  return 1;
}



void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
   digitalWrite(LED_BUILTIN, HIGH); 
}

void softReset(){
asm volatile ("  jmp 0");
}
