#include <Arduino.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include "encoder.h"
#include "configServer.h"
#include "bed.h"
#include "encodedDrivetrain.h"
#include "ota.h"
#include "arm.h"
#include "taskSequence.h"
#include "motor_control.h"
#include "tape_following.h"
#include <ArduinoOTA.h>
#include "operations.h"
#include "robotMessenger.h"
#include "main.h"
#include "rgbled.h"
#include "buttonPanel.h"

#define PIN_SDA 46
#define PIN_SCL 3

#define START_TO_PLATES_TICKS 964
#define PLATES_TO_CHEESE_TICKS 964
#define TOMATO_TO_COOKTOP_TICKS 752
#define COOKTOP_TO_LETTUCE_TICKS 201
#define COOKTOP_TO_SERVING_TICKS 350
#define TOMATO_TO_SERVING_TICKS 550
#define SERVING_TO_PLATES_TICKS 550
#define DONT_CARE_TICKS 100000

#define BOTTOM_BUN_RETRACT_DISTANCE 210
#define CHEESE_RETRACT_DISTANCE 210
#define TOMATO_RETRACT_DISTANCE 220
#define PATTY_RETRACT_DISTANCE 200
#define LETTUCE_RETRACT_DISTANCE 230 
#define MAX_RETRACT_DISTANCE 10000 // it automatically stops from the limit switch instead.

#define DEFAULT_FORWARD_AMOUNT 25

void go(double distance) {
}

void cheesePlate(bool fromStart) { // 1 point
  if (fromStart) {
    // drive plate
    drive_to_relative_station(LEFT, 3, START_TO_PLATES_TICKS);
  }
  else {
    // drive plate
    drive_to_relative_station(LEFT, 2, SERVING_TO_PLATES_TICKS);
  }
  
  // pickup plate
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 136, 0.0, LEFT, 30, MAX_RETRACT_DISTANCE);

  // drive to cheese
  drive_to_relative_station(LEFT, 1, PLATES_TO_CHEESE_TICKS);

  // pickup cheese
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 136, 0.1, LEFT, 30, MAX_RETRACT_DISTANCE);
  
  // drive to serving station
  addWaitingTask([](){follow_main_line(TAPE_FOLLOW_SPEED, 0.075, TOMATO_TO_SERVING_TICKS); relaxBed();}, [](){return min(getLeftEncoderPosition(), getRightEncoderPosition()) > TOMATO_TO_SERVING_TICKS;});

  // serve!
  serve(RIGHT);

  exit_serving(LEFT);
}

void salad(bool fromStart) { // 2 points
  if (fromStart) {
    // drive plate
    drive_to_relative_station(LEFT, 3, START_TO_PLATES_TICKS);
  }
  else {
    // drive plate
    drive_to_relative_station(LEFT, 2, SERVING_TO_PLATES_TICKS);
  }
  
  // pickup plate and lettuce
  _180_pick_up(DEFAULT_FORWARD_AMOUNT, LEFT, 136, -0.1, 25, MAX_RETRACT_DISTANCE, 147, 0.2, 20, MAX_RETRACT_DISTANCE); //_180_pick_up(LEFT, 142, 0, 25, MAX_RETRACT_DISTANCE, 143, 0.0, 30, MAX_RETRACT_DISTANCE);
  
  // drive to tomato
  drive_to_relative_station(RIGHT, 3, PLATES_TO_CHEESE_TICKS);
  
  // pickup tomato
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, RIGHT, 145, 0.2, RIGHT, 30, MAX_RETRACT_DISTANCE);

  // drive to serving station
  addWaitingTask([](){follow_main_line(TAPE_FOLLOW_SPEED, 0.075, TOMATO_TO_SERVING_TICKS); relaxBed();}, [](){return min(getLeftEncoderPosition(), getRightEncoderPosition()) > TOMATO_TO_SERVING_TICKS;});

  // serve!
  serve(RIGHT);


  // get out of serving station
  exit_serving(LEFT);
}

void twoSalad() { // 4 points
  salad(true);
  salad(false);
}

void fries(bool fromStart) { // 5 points

  if (fromStart) {
    // drive plate
    drive_to_relative_station(LEFT, 3, START_TO_PLATES_TICKS);
  }
  else {
    // drive plate
    drive_to_relative_station(LEFT, 2, SERVING_TO_PLATES_TICKS);
  }
  
  // pickup plate
  pick_up_at_station(15, LEFT, 136, 0.05, LEFT, 30, MAX_RETRACT_DISTANCE);

  // drive to cooktop
  drive_to_relative_station(RIGHT, 1, COOKTOP_TO_LETTUCE_TICKS);

  // pickup fries
  pick_up_at_station_fries(DEFAULT_FORWARD_AMOUNT, RIGHT, 140, 0.15, LEFT, 30, MAX_RETRACT_DISTANCE);

  // drive to serving station
  addWaitingTask([](){follow_main_line(TAPE_FOLLOW_SPEED, 0.075, COOKTOP_TO_SERVING_TICKS); relaxBed();}, [](){return min(getLeftEncoderPosition(), getRightEncoderPosition()) > COOKTOP_TO_SERVING_TICKS;});

  // serve!
  serve(LEFT); 

  exit_serving(LEFT);
}

void saladAndFries() { // 7 points
  salad(true);
  fries(false);
}

void burger() { // 7 points
  // drive to plates
  drive_to_relative_station(LEFT, 3, DONT_CARE_TICKS);

  // pickup plates and bottom bun
  smart_plate_and_bottom_bun_pick_up(DEFAULT_FORWARD_AMOUNT, 142, -0.1, 25, 0.35, 0, 200);

  // drive to cooktop
  drive_to_relative_station(RIGHT, 1, DONT_CARE_TICKS);

  // pickup patty and top bun
  smart_top_bun_pick_up(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.35, LEFT, 0, 200);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075, DONT_CARE_TICKS);}, 1000);

  // serve!
  serve(LEFT);

  exit_serving(LEFT);
}

void deluxeCheeseBurger() { // 12 points
  // drive to plates
  drive_to_relative_station(LEFT, 3, DONT_CARE_TICKS);

  // pickup plates and bottom bun
  smart_plate_and_bottom_bun_pick_up(DEFAULT_FORWARD_AMOUNT, 135, -0.1, 25, 0.35, 0, 200);
  
  // drive to cheese
  drive_to_relative_station(LEFT, 1, PLATES_TO_CHEESE_TICKS);

  // pickup cheese and tomato
  addTimedTask([](){sendRobotMessage(PATTY_READY);}, 0);
  _180_pick_up(DEFAULT_FORWARD_AMOUNT, LEFT, 136, 0.35, 0, MAX_RETRACT_DISTANCE, 147, 0.35, 0, MAX_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(LEFT, 2, TOMATO_TO_COOKTOP_TICKS);
  
  // pickup patty
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 143, 0.35, RIGHT, 0, PATTY_RETRACT_DISTANCE);
  
  // drive to lettuce
  drive_to_relative_station(RIGHT, 1, COOKTOP_TO_LETTUCE_TICKS);

  //pickup lettuce
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.35, RIGHT, 0, LETTUCE_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(RIGHT, 1, COOKTOP_TO_LETTUCE_TICKS);

  //pickup top bun
  smart_top_bun_pick_up(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.35, LEFT, 0, 200);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075, COOKTOP_TO_SERVING_TICKS); relaxBed();}, 1700);

  // serve!
  serve(LEFT);

  // leave serving station
  exit_serving(LEFT);
}

void deluxeCheeseBurgerAndFries(bool fromStart) { // 19 points
  if (fromStart) {
    // drive plate
    drive_to_relative_station(LEFT, 3, START_TO_PLATES_TICKS);
  }
  else {
    // drive plate
    drive_to_relative_station(LEFT, 2, SERVING_TO_PLATES_TICKS);
  }

  // pickup plates and bottom bun
  smart_plate_and_bottom_bun_pick_up(15, 138, -0.1, 30, 0.38, 2, BOTTOM_BUN_RETRACT_DISTANCE);
  
  // drive to cheese
  drive_to_relative_station(LEFT, 1, PLATES_TO_CHEESE_TICKS);

  // pickup cheese and tomato
  // addTimedTask([](){sendRobotMessage(PATTY_READY);}, 0);
  _180_pick_up(20, LEFT, 142, 0.35, 0, CHEESE_RETRACT_DISTANCE, 152, 0.35, 0, TOMATO_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(LEFT, 2, TOMATO_TO_COOKTOP_TICKS);
  
  // pickup patty
  smart_patty_pick_up(DEFAULT_FORWARD_AMOUNT, LEFT, 151, 0.38, RIGHT, 10, PATTY_RETRACT_DISTANCE);
  
  // drive to lettuce
  drive_to_relative_station(RIGHT, 1, COOKTOP_TO_LETTUCE_TICKS);

  //pickup lettuce
  // addTimedTask([](){sendRobotMessage(FRIES_READY);}, 0);
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, RIGHT, 147, 0.33, RIGHT, 0, LETTUCE_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(RIGHT, 1, COOKTOP_TO_LETTUCE_TICKS);
  
  // pickup top bun and fries
  smart_fries_and_top_bun_pick_up(10, 152, 0.2, 15, 0.35, 0, 200);

  // drive to serving station
  addWaitingTask([](){follow_main_line(TAPE_FOLLOW_SPEED, 0.075, COOKTOP_TO_SERVING_TICKS); relaxBed();}, [](){return min(getLeftEncoderPosition(), getRightEncoderPosition()) > COOKTOP_TO_SERVING_TICKS;});

  // serve!
  serve(LEFT); 

  // leave serving station
  exit_serving(LEFT);
}

void deluxeCheeseBurgerFriesAndSalad() { // 22 points
  // drive to plates
  drive_to_relative_station(LEFT, 3, DONT_CARE_TICKS);

  // pickup plates and bottom bun
  smart_plate_and_bottom_bun_pick_up(DEFAULT_FORWARD_AMOUNT, 142, -0.1, 25, 0.35, 0, 200);
  
  // drive to cheese
  drive_to_relative_station(LEFT, 1, DONT_CARE_TICKS);

  // pickup cheese and tomato
  _180_pick_up(DEFAULT_FORWARD_AMOUNT, LEFT, 136, 0.2, 20, CHEESE_RETRACT_DISTANCE, 143, 0.2, 15, CHEESE_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(LEFT, 2, DONT_CARE_TICKS);
  
  // pickup patty
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 143, 0.2, RIGHT, 15, PATTY_RETRACT_DISTANCE);
  
  // drive to lettuce
  drive_to_relative_station(RIGHT, 1, DONT_CARE_TICKS);

  //pickup lettuce
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.2, RIGHT, 35, LETTUCE_RETRACT_DISTANCE);
  
  // drive to cooktop
  drive_to_relative_station(RIGHT, 1, DONT_CARE_TICKS);

  // pickup top bun and fries
  smart_top_bun_pick_up(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.35, LEFT, 0, 200);

  // drive to lettuce
  drive_to_relative_station(RIGHT, 1, DONT_CARE_TICKS);

  //pickup lettuce
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, RIGHT, 143, 0.2, RIGHT, 25, LETTUCE_RETRACT_DISTANCE);

  // drive to tomato
  drive_to_relative_station(LEFT, 1, DONT_CARE_TICKS);

  //pickup tomato
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 143, 0.2, LEFT, 25, TOMATO_RETRACT_DISTANCE);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075, DONT_CARE_TICKS); relaxBed();}, 2000);

  // serve!
  serve(RIGHT); 

  exit_serving(LEFT);
}

void deluxeCheeseBurgerAndFriesThenSalad() { // 21 points
  deluxeCheeseBurgerAndFries(true);
  salad(false);
}

void deluxeCheeseBurgerAndFriesThenFries() { // 24 points
  deluxeCheeseBurgerAndFries(true);

  fries(false);
}

void setup() {
  // Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);

  initEncodedDrivetrain();
  init_motor_control();

  addTweakableVariable("GO", go, 0);
  addTweakableVariable("Reboot", [](double val){ESP.restart();}, 0);
  addTweakableVariable("Run to relative position (0.6 speed) (typical range ~100)", [](double dist){driveStraightToPosition(dist, 0.6);}, 0);

  addTweakableVariable("Swing turn right (0.6 speed) (typical range ~100)", [](double dist){swingTurnRight(dist, 0.6);}, 0);
  addTweakableVariable("Swing turn left (0.6 speed) (typical range ~100)", [](double dist){swingTurnLeft(dist, 0.6);}, 0);

  addTweakableVariable("Move Bed (typical range ~0.2)", [](double position){activePositionBed(position);}, 0);
  addTweakableVariable("Rotate Arm (typical range ~0.1)", [](double position){activePositionArm(position);}, 0);

  addTweakableVariable("Extend Arm", [](double val){extendArm(val);}, 0);
  addTweakableVariable("Retract Arm", [](double val){retractArm(val);}, 0);
  addTweakableVariable("Relax Arm Rotation", [](double val){relaxArmRotation();}, 0);
  addTweakableVariable("Relax Bed", [](double val){relaxBed();}, 0);

  // addTweakableVariable("Follow Main Line (0.1 turning)", [](double val){follow_main_line(val, 0.1);}, 0);
  addTweakableVariable("Stop Follow Main Line", [](double val){stop_following_main_line();}, 0);

  addTweakableVariable("Run Motors Forward", [](double val){steer_drivetrain(val, 0);}, 0);

  addValueReadout("millis", [](){return (double) millis();});
  addValueReadout("Front left", [](){return (double) analogRead(6);});
  addValueReadout("Front right", [](){return (double) analogRead(8);});
  addValueReadout("Left", [](){return (double) analogRead(4);});
  addValueReadout("Right", [](){return (double) analogRead(7);});
  addValueReadout("Center", [](){return (double) analogRead(5);});

  initRgbLed();
  initButtonPanel();
   
  #ifndef WIFI_DISABLED
  initConfigServer();
  initOTA();
  #endif
   
  initArm(); // must comebefore the bed
  initBed();
  init_tape_following(3000);
  initRobotMessenger();
  
  // start position
  addWaitingTask([](){}, [](){return !digitalRead(0);});
  addTimedTask([](){extendArm(0.6); activePositionArm(-1.1);}, 2000);

  // start button
  addWaitingTask([](){}, [](){return !digitalRead(0);});

  /*drive_to_relative_station(LEFT, 3, START_TO_PLATES_TICKS);
  
  // pickup plate
  pick_up_at_station(DEFAULT_FORWARD_AMOUNT, LEFT, 143, 0.05, LEFT, 30, MAX_RETRACT_DISTANCE);*/

  // tasks
  // twoSalad();
}

#define TICK_RATE_MS 5

void loop() {
  static long nextTickTime = millis();
  
  if (millis() >= nextTickTime) {
    /*Serial.printf("Left: %d, Right: %d ", LeftEncoder.position, RightEncoder.position);
    Serial.printf("A: %d, B: %d ", digitalRead(ENCODER_RIGHT_PIN_A), digitalRead(ENCODER_RIGHT_PIN_B));
    Serial.printf("\n");*/
    tickDrivetrain();
    tickBed();
    tickArm();
    tickTapeFollowing();
    tickButtonPanel();

    tickTaskSequence();

    #ifndef WIFI_DISABLED
    ArduinoOTA.handle();
    #endif

    Serial.printf("%d ", analogRead(FRONT_LEFT_SENSOR));
    Serial.printf("%d \n",millis());

    //delay(20);

    nextTickTime += TICK_RATE_MS;
  }
}
