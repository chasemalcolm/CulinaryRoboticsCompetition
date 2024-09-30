#include <Arduino.h>


#include "robotMessenger.h"
/* CODE FOR CHIP A - will be replaced by cooking bot */
#include <WiFi.h>
#define SWITCH_PIN 0
#define LED_PIN 4


// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Honor 6X";
const char* password = "1234512345";

// in competition code, this would be handled by another module.
void initWifiStuff() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // The only reason we connect to Wi-Fi is to ensure we are on the same WiFi channel as the receiving ESP
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  initWifiStuff();
  initRobotMessenger();
}

void loop() {
  static Phase phase = START;
  if (!digitalRead(SWITCH_PIN)) {
    phase = (Phase) ((int) phase + 1); //this is terrible code
    sendRobotMessage(phase);
    delay(1000);
  }
}


