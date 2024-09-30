#include <Arduino.h>
#define PULSE_PIN PB12
#define PULSE_FREQ_HZ 2500


void setup() {
  // put your setup code here, to run once:
  pinMode(PULSE_PIN, OUTPUT);
  tone(PULSE_PIN, PULSE_FREQ_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
}