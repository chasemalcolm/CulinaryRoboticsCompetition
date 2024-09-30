#include "buttonPanel.h"
#include "main.h"
#include "rgbled.h"
#include "configServer.h"
#include <Arduino.h>

const int buttonPins[] = {42, 39, 45, 2}; // the 4th is connected to io48 but that'sthe RGB LED sowe can't use it
#define NUM_BUTTONS 4

long lastButtonPressedMillis; // the first button press will always be much later than this
#define BUTTON_PRESS_TIMEOUT_MS 100 // no buttons must be pressed for this long to read a new button press

RGBColor currentColor = {0, 0, 0};

bool drivingFromStartingStation = true;

void initButtonPanel() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
    lastButtonPressedMillis = millis();

    addValueReadout("Button 0", [](){return (double) digitalRead(buttonPins[0]);});
    addValueReadout("Button 1", [](){return (double) digitalRead(buttonPins[1]);});
    addValueReadout("Button 2", [](){return (double) digitalRead(buttonPins[2]);});
    addValueReadout("Button 3", [](){return (double) digitalRead(buttonPins[3]);});
}

// the buttons here are numbered 0-3
void handleButtonPress(int buttonIndex) {
    switch (buttonIndex) {
        case 0:
            currentColor = addColor(currentColor, {10, 10, 0}); // yellow
            setLedColor(currentColor); 
            cheesePlate(drivingFromStartingStation);
            break;
        case 1:
            currentColor = addColor(currentColor, {0, 10, 10}); // blue-green to differentiate from the pure green at boot
            setLedColor(currentColor); 
            salad(drivingFromStartingStation);
            break;
        case 2:
            currentColor = addColor(currentColor, {10, 4, 4}); // pink
            setLedColor(currentColor); 
            deluxeCheeseBurgerAndFriesThenSalad();
            break;
        case 3:
            currentColor = addColor(currentColor, {10, 0, 10}); // purple
            setLedColor(currentColor);
            deluxeCheeseBurgerAndFriesThenFries();
        default: // nothing
            break;
    }
    drivingFromStartingStation = false; // don't come from starting station next time
}

void tickButtonPanel() {
    bool buttonPressed = false;
    int buttonpressedIndex;
    // read all the button pins
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (digitalRead(buttonPins[i]) == LOW) {
            buttonPressed = true;
            buttonpressedIndex = i;
        }
    }

    if (buttonPressed) {
        if (millis() > lastButtonPressedMillis + BUTTON_PRESS_TIMEOUT_MS) {
            handleButtonPress(buttonpressedIndex);
        }
        lastButtonPressedMillis = millis();
    }
}