/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */
//#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define ESP32
#define hostname "pypilot_remote"
WiFiClient client;
#define USING_EUROPE true

const char* host = "10.10.10.1";
const char* ssid = "openplotter";
const char* password = "12345678";


/* Pypilot Status*/

//extern int wifi_event;
extern bool ap_enabled;
extern bool ap_enable_event;
extern char ap_heading[10];
extern char ap_heading_command[10];
extern int ap_mode_e;
extern bool ap_mode_event;
//extern char ap_pilot[10];
//extern char ap_preferred_mode[20];
//extern char ap_tack_direction[10];
extern char servo_command[10];
extern char servo_controller[10];
extern char servo_flags[10];
extern int wifi_read_str_py();
extern char head_str[40];
extern int head_event;
extern char servo_str[40];
extern int servo_event;
extern bool tack_event;
extern int tack_stat;
extern bool tack_enable;
extern void setup_py();


int wifi_stat=0;  // 0 = not connected to WiFi , 1 = Connectting to PyPylot , 2 = init PyPilot , 3 = Connected
String line;


//bool connected = 0;
//extern void setup_lcd();
//extern void disp_text(String line);
extern char STR1[];


// Capture WiFi event

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
        case ARDUINO_EVENT_WIFI_READY: 
            Serial.println("WiFi interface ready");
            break;
        case ARDUINO_EVENT_WIFI_SCAN_DONE:
            Serial.println("Completed scan for access points");
            break;
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println("WiFi client started");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Serial.println("WiFi clients stopped");
            wifi_stat=0;
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("Connected to access point");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            wifi_stat=0;
      
            break;
        default: 
        Serial.print("default event" );
        Serial.println(event);
        break;
    }
    }


// Connetc to WiFi

void wifi_setup() {

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

    // delete old config
    
        WiFi.disconnect(true);

    delay(1000);

   WiFi.onEvent(WiFiEvent);
  WiFi.setHostname(hostname);
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
wifi_stat=1;
}


/*
Main WiFi loop receive data from PyPilot and update event running in a Core
*/

int wifi_loop() {

  // Use WiFiClient class to create TCP connections
  int r;
if(wifi_stat==0){
wifi_setup();
}
  if (wifi_stat == 1) {
    const int httpPort = 23322;
    if (!client.connect(host, httpPort)) {
      Serial.println("connection failed");
      //          disp_text("failed");
      wifi_stat = 0;
      return 0;
    }
  wifi_stat=2;
  }
/*
  if (wifi_stat == 2) {
wifi_stat=3;
    client.print("watch={\"ap.heading\":1.0}\n");
    wifi_read_str_py();
    client.print("watch={\"ap.mode\":true}\n");
    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.tack.timeout\":0.25}\n");
    wifi_read_str_py();
    client.print("watch={\"ap.tack.state\":true}\n");
    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.tack.direction\":true}\n");
    wifi_read_str_py();
    client.print("watch={\"ap.heading_command\":0.5}\n");
    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.pilot\":true}\n");
    wifi_read_str_py();
    client.print("watch={\"servo.controller\":true}\n");
    wifi_read_str_py();
    delay(100);
    client.print("watch={\"servo.flags\":true}\n");
    wifi_read_str_py();
    client.print("watch={\"ap.enabled\":true}\n");
delay(100);
      

  }
*/

  wifi_read_str_py();
  return r;
}


void send_init(){
  
  if (wifi_stat == 2) {
wifi_stat=3;
    client.print("watch={\"ap.heading\":1.0}\n");
//    wifi_read_str_py();
    client.print("watch={\"ap.mode\":true}\n");
//    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.tack.timeout\":0.25}\n");
//    wifi_read_str_py();
    client.print("watch={\"ap.tack.state\":true}\n");
//    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.tack.direction\":true}\n");
//    wifi_read_str_py();
    client.print("watch={\"ap.heading_command\":0.5}\n");
//    wifi_read_str_py();
    delay(100);
    client.print("watch={\"ap.pilot\":true}\n");
//    wifi_read_str_py();
    client.print("watch={\"servo.controller\":true}\n");
//    wifi_read_str_py();
    delay(100);
    client.print("watch={\"servo.flags\":true}\n");
//    wifi_read_str_py();
    client.print("watch={\"ap.enabled\":true}\n");
delay(100);
  }     
}



int wifi_read_str_py() {
  int str_len;

  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
wifi_stat=0;    
  }
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 20) {
      //           Serial.println(">>> Client Timeout !");
 //     ced=0;
      //        client.stop();
      return 0;
    }
  }

  //while(1){
  // Read all the lines of the reply from server and print them to Serial
  //  while (client.available()) {
  //      Serial.println("i client while");
  //        String line = client.readStringUntil('\n');
  line = client.readStringUntil('\n');
  //        Serial.println("i client while efter read");
  str_len = line.length() + 1;
  line.toCharArray(STR1, str_len);
  //           Serial.println(STR1);
  //           Serial.println("rad\n");
  //  }

  //}


  switch (STR1[3]) {

    case 0x76:  //"v" servo
      line.toCharArray(servo_str, str_len);
      servo_event = 1;
      break;

    case 0x68:  //"h" ap.heading
      line.toCharArray(head_str, str_len);
      head_event = 1;
      break;

    case 0x74:  // "t" ap.tack.direction=
    //  line.toCharArray(tack_str, str_len);
    tack_stat=STR1[15];
      tack_event = 1;
      break;

    case 0x65:  // "e" ap.enabled

      if (STR1[11] == 0x74) {  //"t" ap.enabled=true
        ap_enabled = 1;
      } else {
        ap_enabled = 0;
      }
      ap_enable_event = 1;
      break;

      case 0x6D:  //"m" ap.mode=
      ap_mode_e=STR1[9];      
      ap_mode_event=1;

      break;

    default:
      Serial.println("default");
      Serial.println(STR1);
      break;  // code block
  }

  return 1;
}

void serv_cmd(int dir_head) {
  int dir;
  char mes[30];

  if (ap_enabled == 0) {
    sprintf(mes, "servo.command=%d\n", dir_head);

  } else {
    dir = atoi(ap_heading_command);
    dir = dir_head + dir;
    if(dir>360){
      dir=dir-360;
    }
    ap_heading[1] = 0x20;
    ap_heading[2] = 0x20;
    itoa(dir, ap_heading_command, 10);
    sprintf(mes, "ap.heading_command=%d\n", dir);
  }
  //Serial.println(mes);
  client.print(mes);
}



void wifi_send(int c) {
  char mes[30];

  switch (c) {
    case 1:
      serv_cmd(-2);
      //      client.print("servo.command=-2\n");

      break;
    case 2:
      setup_py();
      break;
    case 3:
      serv_cmd(2);
      //      client.print("servo.command=2\n");

      break;
    case 4:
  //      client.print("servo.command=-10\n");
    if( (ap_enabled + tack_enable) >1 ){
    Serial.println("tack + ap enabled port");
    client.print("ap.tack.direction=\"port\"\n");
    } else {
      serv_cmd(-10);
    }
      break;
    case 5:
      if (ap_enabled == 0) {
        client.print("ap.enabled=1\n");
        ap_heading_command[0] = ap_heading[0];
        ap_heading_command[1] = ap_heading[1];
        ap_heading_command[2] = ap_heading[2];
        sprintf(mes, "ap.heading_command=%s\n", ap_heading_command);
        delay(10);
        client.print(mes);
        //     ap_enabled=1;
        //     ap_enable_event=1;
      } else {
        client.print("ap.enabled=0\n");
        delay(10);
        ap_heading_command[0] = 0x20;
        ap_heading_command[1] = 0x20;
        ap_heading_command[2] = 0x20;
        //     ap_enabled=0;
        //     ap_enable_event=1;
      }
      break;

    case 6:
      //      client.print("servo.command=10\n");
    if( (ap_enabled + tack_enable) >1 ){
    Serial.println("tack + ap enabled starboard");
    client.print("ap.tack.direction=\"starboard\"\n");
    } else {
      serv_cmd(10);
    }

//      serv_cmd(10);
      break;

    case 11:
client.print("ap.mode=\"gps\"\n");
break;     

    case 14:
client.print("ap.mode=\"compass\"\n");
break;     

    case 13:
client.print("ap.mode=\"wind\"\n");
break;     

    case 16:
client.print("ap.mode=\"true wind\"\n");
break;     

    default:
      Serial.println(c);
      break;
  }
  //  delay(1000);
  //  wifi_loop();
  //  client.print("servo.command=0\n");
  //wifi_loop();
}


/*
ap.heading_command=338.0280
watch={"ap.pilot":true}
watch={"servo.controller":true}
watch={"ap.heading":0.5}
watch={"ap.heading":2.0}

watch={"ap.pilot":true}
watch={"servo.controller":true}
watch={"ap.mode":true}
watch={"ap.tack.timeout":0.25}
watch={"ap.tack.state":true}
watch={"ap.tack.direction":true}
watch={"ap.heading_command":0.5}
watch={"ap.pilot":true}
watch={"imu.heading_offset":0.5}
watch={"imu.alignmentQ":0.5}
watch={"imu.alignmentCounter":0.25}
watch={"imu.compass_calibration_locked":true}
watch={"rudder.offset":true}
watch={"rudder.scale":true}
watch={"rudder.nonlinearity":true}
watch={"rudder.range":true}
watch={"nmea.client":true}
watch={"servo.controller":true}
watch={"servo.flags":true}
watch={"ap.heading":0.5}


watch={"ap.pilot":true}
  watch={"servo.controller":true}
watch={"ap.mode":true}
watch={"ap.tack.timeout":0.25}
watch={"ap.tack.state":true}
watch={"ap.tack.direction":true}
watch={"ap.heading_command":0.5}
watch={"ap.pilot":true}ap.pilot="basic"
servo.controller="arduino"
ap.mode="compass"
ap.tack.timeout=0
ap.tack.state="none"
ap.tack.direction="starboard"
ap.heading_command=3.3540

ap.tack.direction="port"
ap.tack.direction="starboard"
ap.mode="gps"
ap.mode="compass"
ap.mode="wind"
ap.mode="compass"
ap.mode="true wind"
ap.mode="compass"
ap.heading_command=23.1660
ap.heading_command=13.1660
ap.tack.state="begin"
ap.tack.state="none"
ap.heading_command=23.2260
ap.tack.state="begin"
ap.tack.state="waiting"
ap.tack.state="tacking"
ap.tack.timeout=0
ap.tack.state="none"
watch={"imu.heading_offset":0.5}
watch={"imu.alignmentQ":0.5}
watch={"imu.alignmentCounter":0.25}
watch={"imu.compass_calibration_locked":true}
watch={"rudder.offset":true}
watch={"rudder.scale":true}
watch={"rudder.nonlinearity":true}
watch={"rudder.range":true}
watch={"nmea.client":true}
watch={"servo.controller":true}
watch={"servo.flags":true}imu.heading_offset=0.0000
imu.alignmentQ=[0.9999467178122079, 0.004431196831106279, 0.009323413068017525, 0.0]
imu.alignmentCounter=0
rudder.offset=0.0
rudder.scale=100.0
rudder.nonlinearity=0.0
rudder.range=45.0000
nmea.client="bianca.router:10110"


ap.enabled = False
ap.heading = 23.568
ap.heading_command = 23.226
ap.mode = compass
ap.pilot = basic
ap.preferred_mode = true wind
ap.tack.angle = 100
ap.tack.direction="starboard"
ap.tack.direction="port"
imu.compass = [59.771, -26.047, 52.725]
imu.heading = 23.568
servo.command = 0
servo.controller = arduino
servo.engaged = False
servo.flags = SYNC






*/
