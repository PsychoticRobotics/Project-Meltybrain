#include <stdint.h>
#include "config.h"
#include "LED.h"

#include <FastLED.h>


// to change:
// two modes:
// normal melty mode
// error display mode
// force picking one.


CRGB leds[numled];

IntervalTimer ledTimer;

// Variables to store the current LED states (16-bit values)
volatile uint16_t statusCode   = 0x0000;

void initLEDs() {
  // doing it here coz.. it's an LED, I guess.
  pinMode(13, OUTPUT);          // this is the Teensy built-in LED
  digitalWriteFast(13, HIGH);   // turning it high so you can see reached here.
  // Configure the LED pins as outputs
  FastLED.addLeds<NEOPIXEL, LEDpin>(leds, numled); 
  // Start the timer to call updateLEDs() regularly
  ledTimer.begin(updateLEDs, 2500);  // was 12,500 microseconds = 12.5 milliseconds <= way too slow @ over 2000 RPM
  Serial.println("LEDs initialized in interval timer.");
}

// Function to set the RGB LED states
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx) {
  // Update the red, green, and blue states with the provided values
  if(idx < 0) return;
  if(idx >= numled) return;
  bool waszero = (idx == 0);
  idx = numled -1 - idx; // fix for me doing it backwards
  CRGB color = CRGB(r,g,b);
  leds[idx]= color;
  if(waszero) 
  {
    leds[idx-1]= color;
    leds[idx-2]= color;
    digitalWriteFast(13, r > 0 ? HIGH : LOW);
  }
}


void updateLEDs() 
{
  static volatile int inUpdate = 0;
  inUpdate++;
  if( inUpdate == 1 )
  {
    FastLED.show();
  }
  inUpdate--;
}

uint16_t curCode = 0;
// this sets like 12 or 15 pixels with a value
void setCode(uint16_t code, uint8_t u_val, uint8_t v_val)
{
  curCode = code;
  uint8_t u = u_val;
  uint8_t v = v_val;
  // ok this is the cool idea i had: #R #G #B is the code, then
  // variable length outputs with #R red leds, a gap, #G green leds, 
  // a gap, and #B blue leds.
  uint8_t r = (code&0700)>>6;
  uint8_t g = (code&0070)>>3;
  uint8_t b = (code&0007)>>0;
  uint8_t idx = 0;
  while(idx < numled)
  {
    for(int k = 0; k < r && k < 5; k++) { setRGB(u,v,v,idx++); }
    setRGB(v,v,v,idx++);
    for(int k = 0; k < g && k < 5; k++) { setRGB(v,u,v,idx++); }
    setRGB(v,v,v,idx++);
    for(int k = 0; k < b && k < 5; k++) { setRGB(v,v,u,idx++); }
    setRGB(v,v,v,idx++);
  }
}

