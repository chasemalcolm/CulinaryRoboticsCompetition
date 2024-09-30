#include "rgbled.h"
#include <Adafruit_NeoPixel.h>

#define RGB_LED_PIN 48
#define NUM_LEDS 1
#define LED_INDEX 0

// Setup one led 
Adafruit_NeoPixel pixel(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

void initRgbLed() {
    pixel.begin();
    setLedColor(LED_GREEN);
}

//use pixel.color to get the color values
void setLedColor(uint32_t color) {
    pixel.setPixelColor(LED_INDEX, color);
    pixel.show();
}

void setLedColor(RGBColor color) {
    setLedColor(pixel.Color(color.r, color.g, color.b));
}