#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
#define PIN_SWITCH PB11
#define BUILTIN_LED PB13
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile bool is_switch_pressed;
void handle_switch_press();


void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  is_switch_pressed = false;
  digitalWrite(BUILTIN_LED, !is_switch_pressed);
  attachInterrupt(digitalPinToInterrupt(PIN_SWITCH), handle_switch_press, CHANGE);
  
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);

  /* Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("hello world!");
  display_handler.display();*/
}

void loop() {
};

void handle_switch_press() {
  static volatile int pressCount = 0;
  pressCount++;
  bool is_switch_pressed = !digitalRead(PIN_SWITCH);
  digitalWrite(BUILTIN_LED, !is_switch_pressed);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.printf("Switch is %d", is_switch_pressed);
  display_handler.display();
}