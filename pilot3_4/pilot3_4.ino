/*
Wi-Fi connected remote controller to Py-Pilot
I use LOLIO D32 with an LCD Olimex ESP32-EVB (that I already hade)
Devided the diffrent the two task per core, display and recive Wi-Fi 

Display works with Adafruit_ILI9341.h , the touchscreen STMPE610 from Olimex

*/



extern void g_setup();  //setup LCD
//extern void g_loop();
//extern void setup_ts();
//extern bool loop_ts();
//extern void rem_setup();
//extern void wifi_setup();
extern int wifi_loop();  // Recive Wi-Fi loop on one core
//extern void wifi_send( int );
extern void pilot_loop();  // Main loop LCD
const int led = 5;

#define VER "2023-03-08 pilot3_4"

void setup() {

  Serial.begin(115200);

  pinMode(led, OUTPUT);
  Serial.println(VER);


  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, NULL, 1);

  delay(500);

  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, NULL, 0);

  delay(500);
}

void Task1code(void* pvParameters) {

  Serial.print("Task1  g_setup running on core ");

  Serial.println(xPortGetCoreID());
  g_setup();
  for (;;) {

    digitalWrite(led, HIGH);

    delay(100);

    digitalWrite(led, LOW);

    delay(100);
    Serial.print("Task1 pilot_loop running on core loop 1 ");
    //   g_loop();
    pilot_loop();
    Serial.println(xPortGetCoreID());
  }
}

void Task2code(void* pvParameters) {
  int st;
  Serial.print("Task2 running wifi on core ");
  Serial.println(xPortGetCoreID());
  //LCD_Small(110, 15, "Connecting" ,  ILI9341_BLACK,ILI9341_WHITE);


  //LCD_Small(110, 15, "Connected " ,  ILI9341_BLACK,ILI9341_WHITE);

  delay(2000);
  for (;;) {

    //Serial.print("Task2 running on core loop 2 ");
    delay(10);
    //  Serial.println(xPortGetCoreID());
    st = wifi_loop();
    //Serial.println(st);
  }
}

void loop() {
  delay(10000);
}
