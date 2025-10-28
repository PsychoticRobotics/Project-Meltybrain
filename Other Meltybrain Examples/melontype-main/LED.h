#ifndef __LED_H__
#define __LED_H__
#include <Arduino.h>
const int numled = 12;
const int ledcols = 16;

void initLEDs();
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx=0);

// set code sets "how many red, how many green and how many blue" to display. if this takes fewer than numled pixels, they repeat. there is a blank space between colors.
void setCode(uint16_t code, uint8_t u_val = 255, uint8_t v_val = 0);
void updateLEDs();

void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void addPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void fadeScreen(uint8_t m, uint8_t d);
#endif 