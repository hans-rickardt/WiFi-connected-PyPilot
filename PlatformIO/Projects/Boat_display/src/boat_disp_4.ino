/*
  An example analogue clock using a TFT LCD screen to show the time
  use of some of the drawing commands with the library.

  For a more accurate clock, it would be better to use the RTClib library.
  But this is just a demo.

  This sketch uses font 4 only.

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################

  Based on a sketch by Gilchrist 6/2/2014 1.0
*/
#include "Wire.h"
//#include "Adafruit_STMPE6102.h"
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "NS2009.h"
//#include <ESP8266WiFi.h>
#include <i2cdetect.h>
// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 290
#define TS_MINY 285
#define TS_MAXX 7520
#define TS_MAXY 7510
#define TS_I2C_ADDRESS 0x4d
#define VER "boat_disp_4 2024-02-26"

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
#define KEY_X 40 // Centre of key
#define KEY_Y 96
#define KEY_W 62 // Width and height
#define KEY_H 30
#define KEY_SPACING_X 18 // X and Y gap
#define KEY_SPACING_Y 20
#define KEY_TEXTSIZE 1   // Font size multiplier
// Create 15 keys for the keypad
char keyLabel[15][5] = {"New", "Del", "Send", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#" };
uint16_t keyColor[15] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE
                        };

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[15];


//Adafruit_STMPE610 ts = Adafruit_STMPE610();
NS2009  ts(false, true);
const uint8_t first = 0x03;
const uint8_t last = 0x77;



#include "Free_Fonts.h"
#define TFT_GREY 0x5AEB
static uint8_t conv2d(const char* p);
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0;                    // for next 1 second timeout

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

bool initial = 1;
extern void clock_setup();
extern void clock_loop();
extern void vind_setup();
extern void vind_loop();
extern void json_setup();
extern void json_loop();
extern void digital_setup();
extern void digital_loop();
extern void demo_loop();

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
extern double compass;

void setup(void) {
  delay(1000);
  Serial.begin (115200);
  
  delay(2000);
  Serial.println(VER);
  tft.init();
  tft.setRotation(0);
  delay(2000);
  Serial.println("efter tft");
  
  
  

  //ts.begin(TS_I2C_ADDRESS);
  // Serial.print(TS_I2C_ADDRESS);
  //  ser_setup();
  delay(2000);
  Serial.println("efter ts");
  

  Serial.println(VER);
  json_setup();
    delay(2000);
  Serial.println("efter json");


  tft.drawCentreString(VER, 120, 40, GFXFF);
  delay(5000);
//backligth not implemented 
   pinMode(16, OUTPUT);
    digitalWrite(16, LOW);
Wire.begin();
  i2cdetect(first, last);
}

void loop() {

//  TS_Point p;
  static  int aktiv_task = 3;
  static  int setup_t = 0;
bool th;
  //  Serial.println("Loop");
  json_loop();
  if ( aktiv_task == 1 ) {
    if ( setup_t == 0 ) {
      compass_setup();
      setup_t = 1;
    }

    compass_loop();
  }

  if ( aktiv_task == 2 ) {
    if ( setup_t == 0 ) {
      clock_setup();
      setup_t = 1;
    }
//demo_loop();
    clock_loop();
  }
  
  if ( aktiv_task == 3 ) {
    if ( setup_t == 0 ) {
      vind_setup();
      setup_t = 1;
    }

    vind_loop();
  }

  if ( aktiv_task == 4 ) {
    if ( setup_t == 0 ) {
      digital_setup();
      setup_t = 1;
    }

    digital_loop();
  }

  
  th=getPoint();
 //   Serial.print(ts.X);
 //   Serial.print(" ");
 //   Serial.println(ts.Y);

  if ( th==1 ) {
    delay(500);
 //   p = ts.getPoint();
     setup_t = 0;
    if ( aktiv_task == 1 )
      aktiv_task = 2;
    else if (aktiv_task == 2)
      aktiv_task = 3;
    else if ( aktiv_task == 3)
      aktiv_task = 4;
    else if ( aktiv_task == 4)
      aktiv_task = 1;

  }
}


void compass_setup() {
  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_RED);
  //tft.fillScreen(TFT_GREEN);
  //tft.fillScreen(TFT_BLUE);
  //tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_GREY);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Adding a background colour erases previous text automatically

  // Draw clock face
  tft.fillCircle(120, 120, 118, TFT_RED);
  tft.fillCircle(120, 120, 110, TFT_BLACK);

  // Draw 12 lines
  for (int i = 0; i < 360; i += 15) {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 114 + 120;
    yy0 = sy * 114 + 120;
    x1 = sx * 100 + 120;
    yy1 = sy * 100 + 120;

    tft.drawLine(x0, yy0, x1, yy1, TFT_RED);
  }

  // Draw 60 dots
  for (int i = 0; i < 360; i += 3) {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 102 + 120;
    yy0 = sy * 102 + 120;
    // Draw minute markers
    tft.drawPixel(x0, yy0, TFT_WHITE);

    // Draw main quadrant dots
    /*    if(i==0 || i==180) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
        if(i==90 || i==270) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    */
    tft.setFreeFont(FSB12);

    if (i == 0 ) tft.drawCentreString("N", x0, yy0 + 8, GFXFF);
    if (i == 180 ) tft.drawCentreString("S", x0, yy0 - 22, GFXFF);
    if (i == 90 ) tft.drawCentreString("E", x0 - 8, yy0 - 12, GFXFF);
    if (i == 270 ) tft.drawCentreString("W", x0 + 16, yy0 - 8, GFXFF);
    tft.setFreeFont(FSB9);
    if (i == 45 ) tft.drawCentreString("NE", x0 - 8, yy0 + 8, GFXFF);
    if (i == 315 ) tft.drawCentreString("NW", x0 + 16, yy0 + 8, GFXFF);
    if (i == 135 ) tft.drawCentreString("SE", x0 - 16, yy0 - 20, GFXFF);
    if (i == 225 ) tft.drawCentreString("SW", x0 + 16, yy0 - 20, GFXFF);

  }

  tft.fillCircle(120, 121, 3, TFT_WHITE);

  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.drawCentreString("Kompass", 120, 260, 4);

  targetTime = millis() + 1000;
}

void compass_loop() {

  if (targetTime < millis()) {
    targetTime += 1000;
    ss++;              // Advance second
    if (ss == 60) {
      ss = 0;
      mm++;            // Advance minute
      if (mm > 59) {
        mm = 0;
        hh++;          // Advance hour
        if (hh > 23) {
          hh = 0;
        }
      }
    }
    ss = compass;
    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = compass;                  // 0-59 -> 0-354
    mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 - includes seconds
    hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg - 90) * 0.0174532925);
    hy = sin((hdeg - 90) * 0.0174532925);
    mx = cos((mdeg - 90) * 0.0174532925);
    my = sin((mdeg - 90) * 0.0174532925);
    sx = cos((sdeg - 90) * 0.0174532925);
    sy = sin((sdeg - 90) * 0.0174532925);

    if (ss == 0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, 120, 121, TFT_BLACK);
      ohx = hx * 62 + 121;
      ohy = hy * 62 + 121;
      tft.drawLine(omx, omy, 120, 121, TFT_BLACK);
      omx = mx * 84 + 120;
      omy = my * 84 + 121;
    }

    // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
    tft.drawLine(osx, osy, 120, 121, TFT_BLACK);
    tft.drawLine(osx, osy, 119, 120, TFT_BLACK);
    tft.drawLine(osx, osy, 121, 122, TFT_BLACK);
    tft.drawLine(osx, osy, 118, 119, TFT_BLACK);
    tft.drawLine(osx, osy, 122, 123, TFT_BLACK);
    tft.drawLine(osx, osy, 117, 118, TFT_BLACK);
    tft.drawLine(osx, osy, 123, 124, TFT_BLACK);

    osx = sx * 70 + 121;
    osy = sy * 70 + 121;
    //      tft.drawLine(osx, osy, 120, 121, TFT_RED);
    //      tft.drawLine(ohx, ohy, 120, 121, TFT_WHITE);
    //      tft.drawLine(omx, omy, 120, 121, TFT_WHITE);
    tft.drawLine(osx, osy, 120, 121, TFT_RED);
    tft.drawLine(osx, osy, 119, 120, TFT_RED);
    tft.drawLine(osx, osy, 121, 122, TFT_RED);
    tft.drawLine(osx, osy, 118, 119, TFT_RED);
    tft.drawLine(osx, osy, 122, 123, TFT_RED);
    tft.drawLine(osx, osy, 117, 118, TFT_RED);
    tft.drawLine(osx, osy, 122, 123, TFT_RED);


    tft.fillCircle(120, 121, 5, TFT_RED);
  }
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

/*
  void ser_setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  }
*/

/*
void demo_loop(void) {

// This is just a draw some data Demo

// Clear Screen
tft.fillScreen(ILI9341_BLACK);
// Set some fancy background
testFastLines(ILI9341_DARKGREY,ILI9341_DARKCYAN);

// Print "current date and time"
tft.setCursor(5,5);
tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
tft.println("29-05-18      11:28"); //TODO: Print the real date and time


// Print "room temperature"
tft.setCursor(85,50);
tft.setTextColor(ILI9341_GREEN);  tft.setTextSize(4);
tft.println("22");//TODO: Print the real room temperature
tft.setCursor(148,50);
tft.println("C");
tft.drawCircle(138, 54, 4, ILI9341_GREEN);
tft.drawCircle(138, 54, 5, ILI9341_GREEN);
tft.setCursor(78,85);
tft.setTextColor(ILI9341_GREEN);  tft.setTextSize(1);
tft.println("ROOM TEMPERATURE");


// Now print Message box wit two yes/no buttons
tft.fillRoundRect(10,120, 220, 190, 8, ILI9341_OLIVE);
tft.drawRoundRect(10,120, 220, 190, 8, ILI9341_WHITE);

tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
tft.fillRoundRect(20,150, 200, 80,8, ILI9341_BLUE);
tft.setCursor(90, 165);
tft.println("Save");
tft.setCursor(40, 190);
tft.println("new settings?");
tft.drawRoundRect(20,150, 200, 80, 8, ILI9341_WHITE);
// Get the choise
bool answer = Get_yes_no();

if (answer == true)
{
  // Some animation while "write to eeprom"
testFilledRects(ILI9341_DARKGREEN,ILI9341_DARKCYAN);
tft.setCursor(80, 150);
tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
tft.println("Done!");
} else   tft.fillScreen(ILI9341_RED);
// fill screen red to show negative choise
delay(1000);
}


unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

bool Get_yes_no(void){
TS_Point p;
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);

    tft.fillRoundRect(20,250, 100, 50,8, ILI9341_RED);
    tft.setCursor(56, 265);
    tft.println("NO");
    tft.drawRoundRect(20,250, 100, 50, 8, ILI9341_WHITE);

    tft.fillRoundRect(120,250, 100, 50,8, ILI9341_GREEN);
    tft.setCursor(144, 265);
    tft.println("YES");
    tft.drawRoundRect(120,250, 100, 50, 8, ILI9341_WHITE);


while (1){
      delay(50);
    p = ts.getPoint();

    if (p.z != 129){


      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
      p.y = 320 - p.y;

      //  tft.fillCircle(p.x, p.y, 5, ILI9341_YELLOW);


    if ((p.y > 250) && (p.y<300)){

      if ((p.x> 20) && (p.x < 220))
            if (p.x>120)
            {
              tft.fillRoundRect(120,250, 100, 50,8, ILI9341_OLIVE);
              tft.setCursor(144, 265);
              tft.println("YES");
              tft.drawRoundRect(120,250, 100, 50, 8, ILI9341_WHITE);

              delay(500);
              return true;
            }
                   else{

                     tft.fillRoundRect(20,250, 100, 50,8, ILI9341_OLIVE);
                     tft.setCursor(56, 265);
                     tft.println("NO");
                     tft.drawRoundRect(20,250, 100, 50, 8, ILI9341_WHITE);

                        delay(500);
                     return false;
                   }

    }

  }
}
}


void drawKeypad()
{
  // Draw the keys
  for (uint8_t row = 0; row < 5; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;

      if (b < 3) tft.setFreeFont(FSB9);
      else tft.setFreeFont(FSB12);

      key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                        KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                        KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                        keyLabel[b], KEY_TEXTSIZE);
      key[b].drawButton();
    }
  }
}
*/

bool getPoint()
{
//  ts.ScanBlocking ();
 // Serial.printf ("Screen touched!\n\r");
//return;
//  Handle_Touch ();
//  while (
  return (  ts.CheckTouched ());
    //);
//  Serial.printf ("Screen released!\n\r");
}