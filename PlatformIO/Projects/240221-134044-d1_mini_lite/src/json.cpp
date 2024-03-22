/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialEvent

  {"winddir":90,"windspeed":9,"compass":135, "tid":4000}
{"winddir":90,"windspeed":9,"compass":135, "tid":4000,"depth":12}
*/
#include <Arduino_JSON.h>
void demoParse();
extern String inputString;         // a String to hold incoming data
extern bool stringComplete;  // whether the string is complete

double compass;
double windspeed;
double winddir;
double tid;
bool tidst = 0;
double depth;

void json_setup() {
  // initialize serial:
//  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void json_loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    ///    Serial.println("String");
  //      Serial.println(inputString);
    demoParse();
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
 //    Serial.println(inputString);
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


void demoParse() {
  ///  Serial.println("parse");
  ///  Serial.println("=====");

  JSONVar myObject = JSON.parse(inputString);

  // JSON.typeof(jsonVar) can be used to get the type of the var
  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing input failed!");
    return;
  }

  ///  Serial.print("JSON.typeof(myObject) = ");
  ///  Serial.println(JSON.typeof(myObject)); // prints: object

  // myObject.hasOwnProperty(key) checks if the object contains an entry for key
  if (myObject.hasOwnProperty("tid")) {
    ///    Serial.print("myObject[\"tid\"] = ");

    ///    Serial.println((double) myObject["tid"]);
    tid = ((double) myObject["tid"]);
    if (tid != 0.00)
      tidst = 1;
  }

  if (myObject.hasOwnProperty("windspeed")) {
    ///    Serial.print("myObject[\"windspeed\"] = ");

    ///    Serial.println((double) myObject["windspeed"]);
    windspeed = ((double) myObject["windspeed"]);
  }

  if (myObject.hasOwnProperty("winddir")) {
    ///    Serial.print("myObject[\"winddir\"] = ");

    ///    Serial.println((double) myObject["winddir"]);
    winddir = ((double) myObject["winddir"]);
  }

  if (myObject.hasOwnProperty("compass")) {
    ///    Serial.print("myObject[\"compass\"] = ");

    ///    Serial.println((int) myObject["compass"]);
    compass = ((int) myObject["compass"]);
  }

  if (myObject.hasOwnProperty("depth")) {
    ///    Serial.print("myObject[\"depth\"] = ");

    ///    Serial.println((int) myObject["depth"]);
    depth = ((double) myObject["depth"]);
  }
  // JSON vars can be printed using print or println
  ///  Serial.print("myObject = ");
  ///  Serial.println(myObject);

  //  Serial.println();
}
