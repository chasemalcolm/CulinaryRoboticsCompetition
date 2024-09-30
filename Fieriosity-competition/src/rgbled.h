#pragma once
#include <Arduino.h>
#include <stdint.h>

// 255 is max here
#define LED_GREEN {0, 10, 0}
#define LED_ORANGE {10, 5, 0}
#define LED_RED {255, 0, 0}

struct RGBColor {uint8_t r; uint8_t g; uint8_t b;};

void initRgbLed();

void setLedColor(RGBColor color);

RGBColor addColor(RGBColor color1, RGBColor color2);