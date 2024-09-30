#include <Arduino.h>
#include "bed.h"
#include <I2CEncoder.h>

#define BED_HOME_MOTOR_POWER 0.2
#define BED_HOME_SWITCH_PIN 9
I2CEncoder bedEncoder;

enum bedState {
    ACTIVE_POSITION,
    IDLE
};

void homeBed() {

}

void initBed() {
    bedEncoder.init(MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
}

void tickBed() {
    Serial.printf("Bed Position %f ", bedEncoder.getPosition());
    Serial.printf("Bed limit switch %d", digitalRead(BED_HOME_SWITCH_PIN));
}