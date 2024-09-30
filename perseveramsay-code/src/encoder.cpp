#include <Arduino.h>
#include "encoder.h"
/*
 * When creating an Encoder, pass a function which calls update() on the encoder, to be used for an ISR.
 * In this case, pin A rising first indicated forward motion
 * To use, initialize the object and then call begin() no earlier than void setup()
*/

Encoder::Encoder(int pinA_, int pinB_, void (*updater_)()) {
    position = 0;
    updater = updater_;
    pinA = pinA_;
    pinB = pinB_;
}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    stateA = digitalRead(pinA);
    stateB = digitalRead(pinB);
    //Serial.printf("Encoder initialized on pins %d, %d. Initial A: %d, Initial B: %d \n", pinA, pinB, stateA, stateB);
    attachInterrupt(pinA, updater, CHANGE);
    attachInterrupt(pinB, updater, CHANGE);
}

void IRAM_ATTR Encoder::update() {
    bool newStateA = digitalRead(pinA);
    bool newStateB = digitalRead(pinB);
    if (newStateA != stateA) {
        if (newStateA == newStateB) { // A rises/falls second
        position--;
        }
        else { // A rises/falls first
        position++;
        }
    }
    if (newStateB != stateB) {
        if (newStateA == newStateB) { // B rises/falls second
        position++;
        }
        else { // B rises/falls first
        position--;
        }
    }

    /*if (newStateA != stateA && newStateB != stateB) {
        Serial.printf("Encoder double step error! \n");
        while (true) {} // crash
    }

    if (newStateA == stateA && newStateB == stateB) {
        Serial.printf("Encoder ghost interrupt error! \n");
        while (true) {} // crash
    }*/

    stateA = newStateA;
    stateB = newStateB;
}