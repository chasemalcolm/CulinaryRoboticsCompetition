#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "motor_control.h"
#include "tape_following.h"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {
  Serial.begin(115200);

  Wire.begin(3,46);

  init_tape_following(2000);

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

}

void loop() {
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.printf("Front left: ");
  display_handler.println(analogRead(6));
  display_handler.printf("\nFront right: ");
  display_handler.println(analogRead(8));
  display_handler.printf("\nL: ");
  display_handler.print(analogRead(7));
  display_handler.printf(", R: ");
  display_handler.println(analogRead(4));
  display_handler.printf("\nCenter: ");
  display_handler.println(analogRead(5));
  display_handler.display();
}