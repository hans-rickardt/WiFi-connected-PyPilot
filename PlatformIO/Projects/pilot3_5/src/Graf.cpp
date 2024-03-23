#define ARDUINO_ESP32D

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Wire.h>
#include "Adafruit_STMPE610.h"
#include "Board_Pinout.h"
#include "Roboto_Medium14pt7b.h"
#include "Roboto_Medium8pt7b.h"

extern void g_setup();
extern void setup_ts();
extern void wifi_setup();
extern void wifi_send(int);
extern void pilot_loop();
extern void remote_lcd();
extern void aktiv_lcd();
extern int get_key();
extern void head();
extern void servo();
extern void tack();
extern void tack_lcd();
extern void send_init();
extern void battery();

extern Adafruit_ILI9341 tft;  //= Adafruit_ILI9341(TFT_CS, TFT_DC);
extern Adafruit_STMPE610 ts;
#define TS_I2C_ADDRESS 0x4d

Adafruit_STMPE610 ts = Adafruit_STMPE610();

// Size of the color selection boxes and the paintbrush size
#define BOXSIZE 40

#ifdef ARDUINO_OLIMEXINO_STM32F3
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
#else
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
#endif


char head_str[50];
int head_event = 0;
char servo_str[50];
int servo_event = 0;
#define LCD_max 0
#define LCD_min 255
#define LCD_time 1000
int lcd_cnt;


char STR1[50] = "";


/* Pypilot Status*/
extern int wifi_stat;
bool ap_enabled = 0;
bool ap_enable_event = 0;
bool ap_mode_event = 0;
int ap_mode_e = 0;
bool tack_event = 0;
bool tack_enable = 0;
int tack_stat = 0;
//int lcd_dimm=1000;

char ap_heading[10];
char ap_heading_command[10] = "   ";
int ap_mode = ILI9341_YELLOW;
int ap_sync = ILI9341_YELLOW;
int ap_tack = ILI9341_WHITE;
char ap_tack_direction[10];
char servo_command[10];
char servo_controller[10];
char servo_flags[10];
int ap_mode_gps = ILI9341_WHITE;
int ap_mode_com = ILI9341_WHITE;
int ap_mode_tac = ILI9341_WHITE;
int ap_mode_wap = ILI9341_WHITE;
int ap_mode_wtr = ILI9341_WHITE;

/*
Init the LCD 
*/

void g_setup() {

  delay(1000);
  Serial.print("Pypilot wifi remote started");
  analogWrite(TFT_BL, LCD_max);
  lcd_cnt = LCD_time;
  pinMode(TFT_DC, OUTPUT);

  tft.begin();
  tft.setRotation(1);
  Wire.begin();

  delay(1000);

  ts.begin(TS_I2C_ADDRESS);
}


// LCD screen background light

void lcd_back(bool back) {
  if (back == 1) {
    analogWrite(TFT_BL, LCD_max);
    lcd_cnt = LCD_time;
  } else {
    analogWrite(TFT_BL, LCD_min);
  }
}

// LCD text Font size 8

void LCD_Text(uint16_t Xpos, uint16_t Ypos, const char *str, uint16_t Color, uint16_t bkColor) {
  tft.setTextColor(Color, bkColor);

  tft.setFont(&Roboto_Medium8pt7b);
  tft.setTextSize(2);
  tft.setCursor(Xpos, Ypos);
  tft.println(str);
}


// LCD text Font size 14
void LCD_Num(uint16_t Xpos, uint16_t Ypos, const char *str, uint16_t Color, uint16_t bkColor) {
  tft.setTextColor(Color, bkColor);
  tft.setFont(&Roboto_Medium14pt7b);
  tft.setTextSize(2);

  tft.setCursor(Xpos, Ypos);
  tft.print(str);
}

// Display set ap_mode display

void ap_mode_set() {
  ap_mode_com = ILI9341_WHITE;
  ap_mode_gps = ILI9341_WHITE;
  ap_mode_wap = ILI9341_WHITE;
  ap_mode_wtr = ILI9341_WHITE;
  switch (ap_mode_e) {
    case 0x63:  //"c" ap.mode="compass"
      ap_mode_com = ILI9341_RED;
      ap_mode = ILI9341_RED;
      break;

    case 0x67:  //"g" ap.mode="gps"
      ap_mode_gps = ILI9341_GREEN;
      ap_mode = ILI9341_GREEN;
      break;

    case 0x77:  //"w" ap.mode="wind"
      ap_mode_wap = ILI9341_BLUE;
      ap_mode = ILI9341_BLUE;
      break;
    case 0x74:  //"t" ap.mode="true wind"
      ap_mode_wtr = ILI9341_BLUE;
      ap_mode = ILI9341_BLUE;
      break;
    default:
      Serial.println("default ap_mode");
      Serial.println(ap_mode_e);
      Serial.println(STR1);
      break;  // code block
  }
}

// Main Display loop

void pilot_loop() {


  unsigned short C;
  char key;
  tft.fillScreen(ILI9341_WHITE);

// Detect and Display Wi-Fi status 

  while (wifi_stat != 3) {
    switch (wifi_stat) {
      case 0:
        LCD_Text(6, 40, "wifi connect", ILI9341_BLACK, ILI9341_YELLOW);
        break;
      case 1:
        LCD_Text(6, 90, "Connect to Pypilot", ILI9341_BLACK, ILI9341_YELLOW);
        break;
      case 2:
        LCD_Text(6, 150, "Init Pypilot", ILI9341_BLACK, ILI9341_YELLOW);
        send_init();
        break;
      case 3:
        LCD_Text(6, 200, "Connected to Pypilot", ILI9341_BLACK, ILI9341_YELLOW);
        break;

      default:
        Serial.println("Wifi status ??");
        break;
    }
  battery();
  delay(500);
  }
  delay(5000);

  tft.fillScreen(ILI9341_BLACK);


  delay(5000);

  remote_lcd();
  delay(5000);


// Loop while Wi-Fi status is connected to Py_Pilot

  while (wifi_stat == 3) {
    delay(50);

// Heading event

    if (head_event != 0) {
      head();
      head_event--;
    }

// Servo event
    if (servo_event != 0) {
      servo();
      servo_event--;
    }

// Tack Event
    if (tack_event != 0) {
      tack();
      tack_event = 0;
    }
    if (ap_mode_event != 0) {
      ap_mode_event = 0;
      ap_mode_set();
      tft.fillRoundRect(0, 220, 105, 10, 9, ap_mode);
    }


// Enable event
    if (ap_enable_event == 1) {
      ap_enable_event = 0;

      if (ap_enabled == 1) {
        if (tack_enable == 1) {
          tack_lcd();             //Tack mode enabled
        } else {
          aktiv_lcd();
        }

      } else {
        remote_lcd();
      }
    }
    key = get_key();


    if (key != 0) {  //Check if touch event
      wifi_send(key);   // Sent key 1-6
    }
  }
}



// Default display 

void remote_lcd() {

  tft.fillScreen(ILI9341_BLACK);
  tft.fillRoundRect(0, 0, 105, 105, 9, ILI9341_RED);
  tft.fillRoundRect(0, 108, 105, 105, 9, ILI9341_RED);

  tft.fillRoundRect(214, 0, 105, 105, 9, ILI9341_GREEN);
  tft.fillRoundRect(214, 108, 105, 105, 9, ILI9341_GREEN);

  tft.fillRoundRect(107, 0, 105, 105, 9, ILI9341_WHITE);
  tft.fillCircle(160, 160, 50, ILI9341_YELLOW);

  LCD_Num(38, 66, "<", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(22, 178, "<<", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(234, 66, " >", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Num(234, 178, ">>", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Num(110, 178, "ON", ILI9341_BLACK, ILI9341_YELLOW);

  tft.fillRoundRect(0, 220, 105, 10, 9, ap_mode);
  tft.fillRoundRect(107, 220, 105, 10, 9, ap_sync);
  tft.fillRoundRect(214, 220, 105, 10, 9, ap_tack);
}


// Pilot Aktive

void aktiv_lcd() {

  tft.fillScreen(ILI9341_BLACK);
  tft.fillRoundRect(0, 0, 105, 105, 9, ILI9341_RED);
  tft.fillRoundRect(0, 108, 105, 105, 9, ILI9341_RED);

  tft.fillRoundRect(214, 0, 105, 105, 9, ILI9341_GREEN);
  tft.fillRoundRect(214, 108, 105, 105, 9, ILI9341_GREEN);

  tft.fillRoundRect(107, 0, 105, 105, 9, ILI9341_WHITE);
  tft.fillCircle(160, 160, 50, ILI9341_YELLOW);

  LCD_Num(38, 66, "2", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(22, 178, "10", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(234, 66, " 2", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Num(234, 178, "10", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Num(110, 178, "OFF", ILI9341_BLACK, ILI9341_YELLOW);

  tft.fillRoundRect(0, 220, 105, 10, 9, ap_mode);
  tft.fillRoundRect(107, 220, 105, 10, 9, ap_sync);
  tft.fillRoundRect(214, 220, 105, 10, 9, ap_tack);
}


// Tack enabled 

void tack_lcd() {

  tft.fillScreen(ILI9341_BLACK);
  tft.fillRoundRect(0, 0, 105, 105, 9, ILI9341_RED);
  tft.fillRoundRect(0, 108, 105, 105, 9, ILI9341_RED);

  tft.fillRoundRect(214, 0, 105, 105, 9, ILI9341_BLUE);
  tft.fillRoundRect(214, 108, 105, 105, 9, ILI9341_BLUE);

  tft.fillRoundRect(107, 0, 105, 105, 9, ILI9341_WHITE);
  tft.fillCircle(160, 160, 50, ILI9341_YELLOW);


  LCD_Num(38, 66, "2", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(22, 178, "TB", ILI9341_BLACK, ILI9341_RED);
  LCD_Num(234, 66, " 2", ILI9341_BLACK, ILI9341_BLUE);
  LCD_Num(234, 178, "TS", ILI9341_BLACK, ILI9341_BLUE);
  LCD_Num(110, 178, "OFF", ILI9341_BLACK, ILI9341_YELLOW);

  tft.fillRoundRect(0, 220, 105, 10, 9, ap_mode);
  tft.fillRoundRect(107, 220, 105, 10, 9, ap_sync);
  tft.fillRoundRect(214, 220, 105, 10, 9, ap_tack);
}


// Setup select Compass / GPS / Wind , Battery status

void setup_lcd() {

  char mes[5];
  tft.fillScreen(ILI9341_WHITE);
  tft.drawRoundRect(0, 0, 105, 105, 9, ILI9341_GREEN);
  tft.drawRoundRect(1, 1, 103, 103, 9, ILI9341_GREEN);
  tft.fillCircle(13, 91, 10, ap_mode_gps);


  tft.drawRoundRect(0, 108, 105, 105, 9, ILI9341_RED);
  tft.drawRoundRect(1, 109, 103, 103, 9, ILI9341_RED);
  tft.fillCircle(13, 199, 10, ap_mode_com);

  tft.drawRoundRect(214, 0, 105, 105, 9, ILI9341_BLUE);
  tft.drawRoundRect(215, 1, 103, 103, 9, ILI9341_BLUE);
  tft.fillCircle(227, 91, 10, ap_mode_wap);


  tft.drawRoundRect(107, 0, 105, 105, 9, ILI9341_BLACK);
  tft.drawRoundRect(108, 1, 103, 105, 9, ILI9341_BLACK);
  tft.fillCircle(120, 91, 10, ap_tack);


  tft.drawRoundRect(214, 108, 105, 105, 9, ILI9341_BLUE);
  tft.drawRoundRect(215, 109, 103, 103, 9, ILI9341_BLUE);
  tft.fillCircle(227, 199, 10, ap_mode_wtr);


  tft.drawRoundRect(108, 1, 103, 103, 9, ILI9341_BLACK);
  tft.fillCircle(160, 160, 50, ILI9341_YELLOW);

  LCD_Text(10, 66, "GPS", ILI9341_BLACK, ILI9341_RED);
  LCD_Text(10, 178, "COM", ILI9341_BLACK, ILI9341_RED);
  LCD_Text(234, 66, "WAP", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Text(234, 178, "WTR ", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Text(130, 66, "TAC", ILI9341_BLACK, ILI9341_GREEN);
  LCD_Text(130, 178, "RET", ILI9341_RED, ILI9341_YELLOW);
  battery();
}

void battery(){
  char mes[10];
  int ADC_VALUE = analogRead(35);           // Read Battery volt convert to %

  int adc_percentage = map(ADC_VALUE, 1600, 2550, 0, 100);
  
  tft.fillRoundRect(105, 200, 85, 40, 9, ILI9341_GREEN);
  sprintf(mes, "B %d   ", adc_percentage);
  LCD_Text(110, 230, mes, ILI9341_BLACK, ILI9341_GREEN);
}

// Setup Display and Send setting to PyPilot
void setup_py() {
  int k = 0;

  setup_lcd();
  while (k == 0) {
    k = get_key();
    delay(100);
  }
  switch (k) {

    case 1:
      wifi_send(k + 10);  //  GPS mode
      break;

    case 2:
      if (tack_enable == 0) {
        tack_enable = 1;
        tack_stat = ILI9341_YELLOW;
        ap_tack = ILI9341_RED;
      } else {
        tack_stat = ILI9341_WHITE;
        ap_tack = ILI9341_WHITE;
        tack_enable = 0;
      }

      tack();   // Aktivate Tack display
      break;

    case 3:
      wifi_send(k + 10);  // Wind mode
      break;

    case 4:
      wifi_send(k + 10);  // Compass mode
      break;

    case 5:
      // return
      break;

    case 6:
      wifi_send(k + 10);  // True Wind mode
      break;
  }
  setup_lcd();
  delay(2000);
  ap_enable_event = 1;
}


/*  
The touchscreen is devided in to 6 position return 1-6 and 0 when no key pressed
1   2   3
4   5   6
*/

int get_key() {
  TS_Point p;
  int key;
  static unsigned long delayStart;

// Screen-saver

  if (delayStart + 1000 > millis()) {
    Serial.println(delayStart);
    Serial.println(millis());
    delayStart = millis();

    return 0;
  }
  //delay(50);

  if (lcd_cnt != 0) {
    lcd_cnt--;
    if (0==lcd_cnt) {
      lcd_back(0);
    }
  }

  p = ts.getPoint();

  if (p.z == 129) {

    if (p.x > 4400) {
      if (p.y > 5400) {
        key = 3;
      } else if (p.y > 2800) {
        key = 2;
      } else {
        key = 1;
      }
    } else {
      if (p.y > 5400) {
        key = 6;
      } else if (p.y > 2800) {
        key = 5;
      } else {
        key = 4;
      }
    }
    delay(200);
    // wait for key release
    while (0 != p.z) {
      p = ts.getPoint();
    }
    //    lcd_cnt=LCD_time;
    lcd_back(1);
    //    Serial.println(key);
    return key;
  }
  return 0;
}


/* Get från heading */
void head() {
  int q = 0;
  int c = 0;
  int e = 0;
  int p = 0;
  //Serial.println("head");
  //Serial.println(head_str);

  for (q = 4; q < strlen(head_str); q++) {

    if (head_str[q] == '=') {
      e = q;
    }
    if (head_str[q] == '.') {
      p = q;
    }
  }
  //printf("\n = %d  . %d cnt %d\n",e,p,q);
  c = 0;
  if (e == 10) {
    for (q = e + 1; q < p; q++) {
      ap_heading[c++] = head_str[q];
      ap_heading[c] = 0;
    }

  } else {
    for (q = e + 1; q < p; q++) {
      ap_heading_command[c++] = head_str[q];
      ap_heading_command[c] = 0;
    }
  }
  tft.fillRoundRect(107, 0, 105, 105, 8, ILI9341_WHITE);
  LCD_Num(107, 46, ap_heading, ILI9341_BLACK, ILI9341_WHITE);
  LCD_Num(107, 96, ap_heading_command, ILI9341_BLACK, ILI9341_WHITE);
}





/*
servo.flags="SYNC ENGAGED"
*/
void servo() {
  int len;

  len = strlen(servo_str);
  if (26 == len) {
    ap_sync = ILI9341_RED;
    //    ap_enabled = 1;
    //    ap_enable_event=1;
  } else if (23 == len) {
    ap_sync = ILI9341_BLACK;
    //    ap_enabled = 0;
    //    ap_enable_event=1;
  } else if (18 == len) {
    ap_sync = ILI9341_GREEN;
  }
  tft.fillRoundRect(107, 220, 105, 10, 8, ap_sync);
}


/*
ap.tack.state="begin"
ap.tack.state="waiting"
ap.tack.state="tacking"
ap.tack.state="none"

tack_stat

*/

void tack() {

  switch (tack_stat) {
    case 0x62:  //ap.tack.state="begin"
      ap_tack = ILI9341_GREEN;
      break;
    case 0x77:
      ap_tack = ILI9341_RED;  //ap.tack.state="waiting"
      break;
    case 0x074:  //ap.tack.state="tacking"
      ap_tack = ILI9341_BLUE;
      break;
    case 0x6e:  //ap.tack.state="none"
      ap_tack = ILI9341_YELLOW;
      break;
    default:
      Serial.print(" ");
      Serial.print(tack_stat);
      Serial.println(" tack");
      break;
  }
  /*
if(ap_tack==ILI9341_WHITE){  
 ap_tack=ILI9341_RED;
tft.fillRoundRect(214, 220, 105, 10, 9, ap_tack);
} else {

 ap_tack=ILI9341_WHITE;
}
*/
  //  ap_enable_event = 1;
}
