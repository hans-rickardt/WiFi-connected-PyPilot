#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "Free_Fonts.h"
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

extern double windspeed;
extern double winddir;
extern double depth;

void digital_setup(void) {

  tft.fillScreen(TFT_GREY);

  tft.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically
  tft.setFreeFont(FSB24);

  tft.drawLine(0, 105, 240, 105, TFT_WHITE);
  tft.drawLine(0, 220, 240, 220, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_GREY);

  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString(" ", 120, 260, 4);

}


void digital_loop() {
  String str = "";

  if (targetTime < millis()) {
    targetTime += 1000;


    //    tft.drawLine(0, 105, 240, 105, TFT_WHITE);
    //    tft.drawLine(0, 220, 240, 220, TFT_WHITE);

    //tft.setFreeFont(FSB24);

    //    tft.setTextColor(TFT_WHITE, TFT_GREY);
    char str[15];
    sprintf(str, "     %.0f gr    ", float(winddir)  );
    tft.drawCentreString(str, 120, 40, GFXFF);
    sprintf(str, "    %.2f m/s    ", windspeed );
    tft.drawCentreString(str, 120, 140, GFXFF);
    sprintf(str, "    %.2f m    ", depth );
    tft.drawCentreString(str, 120, 250, GFXFF);

  }
}

/*
  static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
  }
*/
