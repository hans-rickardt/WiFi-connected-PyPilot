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

#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library

#define TFT_GREY 0x5AEB

//TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
extern TFT_eSPI tft;
extern float sx, sy , mx, my , hx , hy;    // Saved H, M, S x & y multipliers
extern float sdeg, mdeg, hdeg;
extern uint16_t osx, osy, omx, omy, ohx, ohy;  // Saved H, M, S x & y coords
extern uint16_t x0, x1, yy0, yy1;
extern uint32_t targetTime ;                    // for next 1 second timeout

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
extern uint8_t hh, mm, ss;  // Get H, M, S from compile time

extern bool initial ;

extern double tid;
extern bool tidst;

void clock_setup(void) {
  //  tft.init();
  //  tft.setRotation(0);



  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_RED);
  //tft.fillScreen(TFT_GREEN);
  //tft.fillScreen(TFT_BLUE);
  //tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_GREY);

  tft.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically

  // Draw clock face
  tft.fillCircle(120, 120, 118, TFT_GREEN);
  tft.fillCircle(120, 120, 110, TFT_BLACK);

  // Draw 12 lines
  for (int i = 0; i < 360; i += 30) {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 114 + 120;
    yy0 = sy * 114 + 120;
    x1 = sx * 100 + 120;
    yy1 = sy * 100 + 120;

    tft.drawLine(x0, yy0, x1, yy1, TFT_GREEN);
  }

  // Draw 60 dots
  for (int i = 0; i < 360; i += 6) {
    sx = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx * 102 + 120;
    yy0 = sy * 102 + 120;
    // Draw minute markers
    tft.drawPixel(x0, yy0, TFT_WHITE);

    // Draw main quadrant dots
    if (i == 0 || i == 180) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    if (i == 90 || i == 270) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
  }

  tft.fillCircle(120, 121, 3, TFT_WHITE);

  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString(" ", 120, 260, 4);

  targetTime = millis() + 1000;
}

void tid_val() {
  int h;
  int m, m1;
  int s;

  if (tidst==1) {
    tidst = 0;
    h = tid / 3600;
    m1 = (tid - (h * 3600));
    m = m1 / 60;
    if (m == 0) {
    }
    s = (m1 - (m * 60));
    h = h + 2;
    if (h > 23) {
      h = h - 24;
      initial = 1;
    }

    hh = h;
    mm = m;
    ss = s;

    //Serial.println("tid");
    //Serial.println(h);
    //Serial.println(m);
    //Serial.println(s);
    tidst = 0;
  }
}

void clock_loop() {
  String str = "";

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

    tid_val();

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss * 6;                // 0-59 -> 0-354
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
    osx = sx * 90 + 121;
    osy = sy * 90 + 121;
    tft.drawLine(osx, osy, 120, 121, TFT_RED);
    tft.drawLine(ohx, ohy, 120, 121, TFT_WHITE);
    tft.drawLine(omx, omy, 120, 121, TFT_WHITE);
 //   tft.drawLine(osx, osy, 120, 121, TFT_RED);

    tft.fillCircle(120, 121, 3, TFT_RED);
    // String   st1 = String(hh , DEC );
    // String   st2 = String(mm , DEC );
    // String   st3 = String(ss , DEC );
    // str = String( "    " + st1 + ":" +  st2 + ":" + st3 +"    " );

    tft.setTextColor(TFT_WHITE, TFT_GREY);
    char str[15];
    sprintf(str, "     %d:%02d    ", hh, mm);
    tft.drawCentreString(str, 120, 260, 6);
  }
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
