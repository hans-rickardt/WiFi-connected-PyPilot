#define CAN_2515
#include <SPI.h>
//#include <mcp2515.h>
//#include <ActisenseReader.h>
//#include <can-serial.h>

//#include "mcp2515_can.h"
//#include <ArduinoTrace.h>
#include <Scheduler.h>

#include <Wire.h>
extern TwoWire Wire1;

extern float temperature;
extern float pressure;
extern float humidity;
extern void rpm_fun();

#include <Adafruit_INA219.h>
#define N2k_SPI_CS_PIN 9  // Pin for SPI Can Select
#define N2k_CAN_INT_PIN 2// Use interrupt  and it is connected to pin 21
#define USE_MCP_CAN_CLOCK_SET 16  // possible values 8 for 8Mhz and 16 for 16 Mhz clock

#define USE_N2K_CAN 1
#include <Arduino.h>

//#include "NMEA2000_CAN.h"  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
#include <N2kMessagesEnumToStr.h>
//#include <mcp_can.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
//#include <EEPROM.h>
#include <DueFlashStorage.h>

#include "Subrutins.h"
#include "BME_680.h"


/*
  Enable Pitch and roll  bno.OPERATION_MODE_NDOF
  enable default bno parameters
*/

/********************************************************************/
#define VER "MultiDevice_hans8_shed3 20230817"
/********************************************************************/

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Analog pin
/*
const int analogMotor = A9;  // Analog input pin water temp
const int analogFluid = A10;  // Analog input pin
const int analogTemp = A11; // Temp sensor
const int analogWater = A8;  // Analog input pin
const int analogVolt = A5;  // Analog input pin
*/
#define analogMotor  A9  // Analog input pin water temp
#define analogFluid  A10  // Analog input pin
#define analogTemp  A11 // Temp sensor
#define analogWater  A8  // Analog input pin
#define analogVolt  A5  // Analog input pin
#define analogToa   A7
//const int analogOutPin = 9; // Analog output pin that the LED is attached to
#define idle_Led  25

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

//compass status LED
//#define LED D9

// List here messages your device will transmit.
const unsigned long TemperatureMonitorTransmitMessages[] PROGMEM = {130310L, 130311L, 130312L, 0};
const unsigned long BatteryMonitorTransmitMessages[] PROGMEM = {127506L, 127508L, 127513L, 0};
const unsigned long MotorTransmitMessages[] PROGMEM = {127488L, 127489L, 0};
//const unsigned long PilotMessages[] PROGMEM = {60928L, 59392L, 59904L, 126996L, 126464L, 127237L, 127245L, 127258L, 127250L, 0};
/*
60928,
  59904,
  59392,
  59904,
  126996,
  127237,
  127245,
  127258,
  127250
*/
/**********************************/
void Wind(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void SystemTime(const tN2kMsg &N2kMsg);
void depth(const tN2kMsg &N2kMsg);
void attitude(const tN2kMsg &N2kMsg);



double compass = 0;
double wdir = 99;
double wspeed = 99;
double tid;
double ddepth = 2;

char buff[20];
typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

tNMEA2000Handler NMEA2000Handlers[] = {
  {126992L, &SystemTime},
  //  {130312L,&Temperature},
//  {130311L, &Temperature},
  {130306L, &Wind},
  {128267L, &depth},
  {127257L, &attitude},
  {0, 0}
};

//Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

/**********************************/

// NMEA 2000 buffer ref
#define DEV_MOT  0
#define DEV_BAT  1
#define DEV_TEMP 2
#define DEV_ALT  3
//
// MX lock for  NMEA2000.SendMsg(N2kMsg, DEV_TEMP);
static int MX;
// Tacho






// I2C SDA1 SCL1 two adresses
// Currnet and Volt messerement
Adafruit_INA219 ina219A(0x40);
Adafruit_INA219 ina219B(0x41);

// Serial Event status flag
int bno_stat = 0;


// Startup

void setup() {
  Serial.begin(115200);
  setup_bme();
  MX = 9; //set mutex lock to free
  // Motor rpm interupt via opto-switch
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm_fun, FALLING     );

  Serial.println(VER);
  delay(10);
  Serial.println("rpmsetup");
  rpmcount = 0;
  rpm = 0;
  timeold = 0;

  //  Analog Setup 12bit resulotion 4096
  analogReadResolution(12);

  //  uint32_t currentFrequency;

  // Initialize the INA219. SDA1 SCL1
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  // setup i2C for current and volt
  // using I2C port Wire1
  ina219A.begin(&Wire1);
  ina219B.begin(&Wire1);
  ina219A.setCalibration_32V_1A();
  ina219B.setCalibration_32V_1A();
  /* Configure NMEA2000*/
  NMEA2000.SetDeviceCount(4); // Enable multi device support for 4 devices
  // Set Product information for temperature monitor

  NMEA2000.SetProductInformation("112233", // Manufacturer's Model serial code.
                                 100, // Manufacturer's product code
                                 "Simple temperature monitor",  // Manufacturer's Model ID
                                 "1.0 (2021-12-04)",  // Manufacturer's Software version code
                                 "1.0 (2021-12-04)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_TEMP
                                );

  // Set Product information for battery monitor
  NMEA2000.SetProductInformation("112234", // Manufacturer's Model serial code.
                                 100, // Manufacturer's product code
                                 "Simple battery monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2021-12-04)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2021-12-04)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_BAT
                                );

  // Set Product information for motor monitor
  NMEA2000.SetProductInformation("112235", // Manufacturer's Model serial code.
                                 100, // Manufacturer's product code
                                 "Simple motor monitor",  // Manufacturer's Model ID
                                 "1.0.0.2 (2021-12-04)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2021-12-04)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_MOT
                                );

  // Set Product information for pilot
  NMEA2000.SetProductInformation("112299", // Manufacturer's Model serial code.
                                 1851, // Manufacturer's product code
                                 "ACU200",  // Manufacturer's Model ID
                                 "v3.04",  // Manufacturer's Software version code
                                 "EV-1", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_ALT
                                );


  // Set device information for temperature monitor
  NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                DEV_TEMP
                               );

  // Set device information for battery monitor
  NMEA2000.SetDeviceInformation(112234,  // Unique number. Use e.g. Serial number.
                                170,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                35,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                DEV_BAT
                               );

  // Set device information for Motor monitor
  NMEA2000.SetDeviceInformation(112235,  // Unique number. Use e.g. Serial number.
                                130,    // Device function=Battery. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                50,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                DEV_MOT
                               );

  // Set device information for 
  NMEA2000.SetDeviceInformation(112299,  // Unique number. Use e.g. Serial number.
                                150,    // Device function=. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                40,     // Device class= . See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040,    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                DEV_ALT
                               );




  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader
  NMEA2000.SetForwardStream(&Serial);

  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);

  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)


  // Here we tell, which PGNs we transmit from battery monitor
  NMEA2000.ExtendTransmitMessages(TemperatureMonitorTransmitMessages, DEV_TEMP);

  NMEA2000.ExtendTransmitMessages(BatteryMonitorTransmitMessages, DEV_BAT);

  NMEA2000.ExtendTransmitMessages(MotorTransmitMessages, DEV_MOT);

 // NMEA2000.ExtendTransmitMessages(PilotMessages, DEV_ALT);

  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  NMEA2000.Open();

  /* Start the Scheduler tasks */
  Scheduler.startLoop(loop2, 6144);
  Scheduler.startLoop(loop3, 6144);
  Scheduler.startLoop(loop4, 6144);
  Scheduler.startLoop(loop5, 6144);

  pinMode(idle_Led, OUTPUT);
}


/*
  void loop6() {
  static int a;
  digitalWrite(idle_Led, LOW);
  a++;
  digitalWrite(idle_Led, HIGH);
  yield();
  }
*/


/* Idle loop flash LED and leave CPU to Scheduler */
void loop() {

  static int a;
  digitalWrite(idle_Led, LOW);
  a++;
  NMEA2000.ParseMessages();
  //  Serial.print("In Main Handler: ");
  digitalWrite(idle_Led, HIGH);
  yield();
}



void loop2() {
  loop_bme();
  SendN2kTemperature();
  delay(500);
  yield();
  send_json();
}


// #define TempUpdatePeriod 1000

void SendN2kTemperature() {

  tN2kMsg N2kMsg;

  //  SetN2kTemperature(N2kMsg, 1, 1, N2kts_OutsideTemperature, CToKelvin(temperature));

  //  SetN2kTemperature(N2kMsg, 1, 1, N2kts_OutsideTemperature, ReadCabinTemp());
  //  NMEA2000.SendMsg(N2kMsg, DEV_TEMP);
  //  delay(200);
  SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_InsideTemperature, CToKelvin(temperature), N2khs_InsideHumidity , humidity,  pressure);

  //  SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_OutsideTemperature, ReadCabinTemp());
  //  NMEA2000.SendMsg(N2kMsg, DEV_TEMP);
  SendMsg(N2kMsg, DEV_TEMP);
}

void SendMsg(tN2kMsg N2kMsg, int DEV) {
  //  Serial.print(DEV);
  //  Serial.println(MX);

  while ( MX != 9) {
    Serial.print("MX inte null ");
    Serial.println(MX);
    delay(10);
  }
  MX = DEV;
  NMEA2000.SendMsg(N2kMsg, DEV);
  MX = 9;
}
//
// Battery info
//

void loop3() {
  SendN2kBattery();
   yield();
  delay(1500);
  yield();

}

#define BatUpdatePeriod 2000

void SendN2kBattery() {
  //  static unsigned long Updated = millis() + 100;
  float mv_A = ina219A.getShuntVoltage_mV() / 2 ;
  float mv_B = ina219B.getShuntVoltage_mV() / 2 ; // shunt motstond
  tN2kMsg N2kMsg;


  SetN2kDCBatStatus(N2kMsg, 0, ina219A.getBusVoltage_V(), mv_A, 294, 1);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);
  SetN2kDCBatStatus(N2kMsg, 1, ina219B.getBusVoltage_V(), mv_B, 295, 1);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);

  SetN2kBatConf(N2kMsg, 0, N2kDCbt_Gel, N2kDCES_Yes, N2kDCbnv_12v, N2kDCbc_LeadAcid, AhToCoulomb(80), 53, 1.251, 85);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);
  SetN2kBatConf(N2kMsg, 1, N2kDCbt_Gel, N2kDCES_Yes, N2kDCbnv_12v, N2kDCbc_LeadAcid, AhToCoulomb(330), 53, 1.251, 85);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);
  //Serial.print("mVolt ");
  //Serial.println(ina219A.getShuntVoltage_mV());
  //Serial.println(ina219B.getShuntVoltage_mV());

  /* */
  sensorValue = analogRead(analogFluid);
  sensorValue = sensorValue - DiffVolt();
  outputValue = map(sensorValue, 3500, 1300, 0, 100); //Test

  SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, outputValue , 0.80);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);



  sensorValue = analogRead(analogWater);
  //Serial.println(sensorValue);
  outputValue = map(sensorValue, 600, 3000, 0, 100); //Test
  //Serial.println(outputValue);
  SetN2kFluidLevel(N2kMsg, 0, N2kft_Water, outputValue , 120);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);

  sensorValue = analogRead(analogToa);
  //Serial.println(sensorValue);
  outputValue = map(sensorValue, 600, 3000, 0, 100); //Test
  //Serial.println(outputValue);
  SetN2kFluidLevel(N2kMsg, 0, N2kft_BlackWater, outputValue , 120);
  //  NMEA2000.SendMsg(N2kMsg, DEV_BAT);
  SendMsg(N2kMsg, DEV_BAT);
  delay(1000);

  /*  Serial.print("Fluid ");
    Serial.println(outputValue);
    Serial.print("Volt ");
    Serial.println(ina219B.getBusVoltage_V());
  */
}


//
// Motor update
//
void loop4() {
  SendN2kMotor();
   yield();
  delay(500);
}


void SendN2kMotor() {
  //  static unsigned long Updated=millis()+200;
  double rpm1;
  double outputValue;
  tN2kMsg N2kMsg;

  rpm1 = getrpm();
  //    Serial.println(rpm1);
  SetN2kEngineParamRapid(N2kMsg, 0, rpm1, 100, 100);
  //  NMEA2000.SendMsg(N2kMsg, DEV_MOT);
  SendMsg(N2kMsg, DEV_MOT);
  delay(200);

  //Analog
  sensorValue = analogRead(analogMotor);
  sensorValue = sensorValue - DiffVolt();

  //    Serial.println(sensorValue);
  outputValue = map(sensorValue, 3270, 890, 0, 100);
  /*  Serial.print(" Motor ");
    Serial.println(outputValue);
  */
  SetN2kEngineDynamicParam(N2kMsg, 0, 100, CToKelvin(outputValue), CToKelvin(outputValue) , 12, 1, 300, 100, 100, 70, 70, 0);
  //  NMEA2000.SendMsg(N2kMsg, DEV_MOT);
  SendMsg(N2kMsg, DEV_MOT);

}



void loop5() {

 tN2kMsg N2kMsg;
//  SetN2kAttitude(N2kMsg, 1, Heading , Pitch, Roll);

  SendMsg(N2kMsg, DEV_ALT);
  delay(100);

//  SetN2kMagneticHeading(N2kMsg, 1, Heading, +0.00, +0.0);

  //  NMEA2000.SendMsg(N2kMsg, DEV_ALT);
  SendMsg(N2kMsg, DEV_ALT);
  delay(100);

}



void event_setup() {
  delay(1000);

  //  Serial.begin(115200);
  //  Serial.println("Teensy 4.0 NMEA2000 display www.skpang.co.uk 12/20");

  //  OutputStream=&Serial;

  // NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  //  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  //  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  //  NMEA2000.EnableForward(false);
  //  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  //  NMEA2000.Open();
  Serial.print("Running...");

  //  skp_lvgl_init();   // Start LVGL

}


/*

bool ParseN2kPGN127257(const tN2kMsg &N2kMsg, unsigned char &SID, double &Yaw, double &Pitch, double &Roll);
inline bool ParseN2kAttitude(const tN2kMsg &N2kMsg, unsigned char &SID, double &Yaw, double &Pitch, double &Roll) {
  return ParseN2kPGN127257(N2kMsg,SID, Yaw, Pitch, Roll);


*/

void attitude(const tN2kMsg &N2kMsg){

  unsigned char SID;
  double Yaw;
  double Pitch;
  double Roll;

  if (ParseN2kAttitude(N2kMsg, SID, Yaw, Pitch, Roll)){
    compass=RadToDeg(Yaw);
//    Serial.println("attiude");
//Serial.println(Yaw);
//Serial.println(Pitch);
//Serial.println(Roll);

  } else {
   Serial.println("Failed to parse attitude PGN: "); Serial.println(N2kMsg.PGN);
  }
}


void depth(const tN2kMsg &N2kMsg) {
  double pi = 3.14;

  unsigned char SID;
  double belowtrans;
  double range;

  double doffset;


  if (ParseN2kWaterDepth(N2kMsg, SID,  belowtrans, doffset, range))
  {

    ddepth = belowtrans + doffset;

  } else {
    Serial.print("Failed to parse PGN: "); Serial.println(N2kMsg.PGN);
  }
}

void Wind(const tN2kMsg &N2kMsg) {
  double pi = 3.14;

  unsigned char SID;
  double WindSpeed;
  double WindAngle;

  double WindAngle_degree;

  tN2kWindReference WindReference;

  if (ParseN2kWindSpeed(N2kMsg, SID,  WindSpeed, WindAngle, WindReference))
  {
    //   Serial.print("Wind speed ");
    //   Serial.print(WindSpeed);
    //   Serial.print(" ");
    wspeed = WindSpeed;

    //   sprintf(buff,"%2.2f",WindSpeed);
    //      lv_label_set_text(label_wind_speed,buff);    // Update wind speed on LCD

    WindAngle_degree = WindAngle * (180 / pi);
    //   Serial.print("Wind direction ");
   //    Serial.println(WindAngle_degree);

    wdir = WindAngle_degree;

    //     lv_img_set_angle(  lvneedle, WindAngle_degree*10);   // Update needle on LCD

  } else {
    Serial.print("Failed to parse PGN: "); Serial.println(N2kMsg.PGN);
  }
}


void Temperature(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char TempInstance;
  tN2kTempSource TempSource;
  tN2kHumiditySource HumSource;
  double ActualTemperature;
  double SetTemperature;
  double temperature;
  double humsource;
  double humvalue;
  double pressure;

  //   if (ParseN2kTemperature(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) )
  //            inline bool ParseN2kTemperature(const tN2kMsg &N2kMsg, unsigned char &SID, unsigned char &TempInstance, tN2kTempSource &TempSource, double &ActualTemperature, double &SetTemperature)
  //inline bool ParseN2kEnvironmentalParameters(const tN2kMsg &N2kMsg, unsigned char &SID, tN2kTempSource &TempSource, double &Temperature, tN2kHumiditySource &HumiditySource, double &Humidity, double &AtmosphericPressure)

  if (ParseN2kEnvironmentalParameters(N2kMsg, SID, TempSource, ActualTemperature, HumSource, humvalue, pressure) )

  {
    temperature = ActualTemperature - 273.15;   // Covert K to C

    Serial.print("Temperature ");
    Serial.println(temperature);
    sprintf(buff, "%2.1f", temperature);
    //        lv_label_set_text(label_temperature, buff);   // Update temperature on LCD

  } else {
    Serial.print("Failed to parse PGN: ");  Serial.println(N2kMsg.PGN);
  }
}

//*****************************************************************************
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val) = 0, bool AddLf = false ) {
  Serial.print(label);
  if (!N2kIsNA(val)) {
    if (ConvFunc) {
      Serial.print(ConvFunc(val));
    } else {
      Serial.print(val);
    }
  } else Serial.print("not available");
  if (AddLf) Serial.println();
}

//*****************************************************************************
void SystemTime(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  uint16_t SystemDate;
  double SystemTime;
  tN2kTimeSource TimeSource;

  if (ParseN2kSystemTime(N2kMsg, SID, SystemDate, SystemTime, TimeSource) ) {


    //      PrintLabelValWithConversionCheckUnDef("System time: ",SID,0,true);

    //      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
    //      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
    //                        Serial.print("  time source: "); PrintN2kEnumType(TimeSource,&Serial);
    tid = SystemTime;
  } else {
    Serial.print("Failed to parse PGN: "); Serial.println(N2kMsg.PGN);
  }
}


//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;

  // Find handler
  //  Serial.print("In Main Handler: "); Serial.println(N2kMsg.PGN);
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

//*****************************************************************************

//inline bool ParseN2kHeading(const tN2kMsg &N2kMsg, unsigned char &SID, double &Heading, double &Deviation, double &Variation, tN2kHeadingReference &ref)
/*
  void Comp(const tN2kMsg &N2kMsg) {

   unsigned char SID;
   double WindSpeed;
   double WindAngle;
   double WindAngle_degree;
  Serial.println("compass");
  }
*/

void send_json() {
  String str1 = "{";
  String str2 = " }";
  String strc = String(compass);
  String strt = String( str1 +  "\"winddir\":" + wdir + ", \"windspeed\":" + wspeed + ", \"compass\":" + strc + ", \"tid\":" + tid + ", \"depth\":" + ddepth  + str2 );
  tid = 0;
  Serial.println(strt);


}
