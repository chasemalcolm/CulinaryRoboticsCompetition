#include <Arduino.h>
#include <ArduinoOTA.h>
#include "encoder.h"
#include "configServer.h"
#include "encodedDrivetrain.h"
#include "ota.h"
#include "motor_control.h"
#include "tape_following.h"
#include "taskSequence.h"
#include "encodedHorizontalSlide.h"
#include "encodedVerticalSlides.h"
#include "robotmessenger.h"
#include "rgbled.h"

// 0.21cm per tick
// 4.6 ticks per cm

#define START_BUTTON 0

#define MAIN_TAPE_FORWARD_POWER 0.5
#define MAIN_TAPE_TURNING_POWER 0.15

#define SIDE_TAPE_FORWARD_POWER 0.4
#define SIDE_TAPE_TURNING_POWER 0.225

#define ENCODER_FORWARD_POWER 0.6

#define NINTY_DEGREE_ON_SPOT_TURN 82
#define ON_SPOT_TURN_POWER 0.65

#define SENSOR_TO_WHEEL_DISTANCE 58

#define TAPE_SENSOR_THRESHOLD 3000

#define SPATULA_MIN_ANGLE -54
#define FRY_PRY_MIN_ANGLE -300
#define FRY_PRY_FRY_WIDTH 140

#define STANDARD_TIME_OUT_TIME 750

int positionFor90degTurn = 142;

void homingSequence() {
  addTimedTask([](){initHorizontalSlide();}, 1500);
  addTimedTask([](){initVerticalSlides();}, 1500);
  addTimedTask([](){activePositionVerticalSlides(75);}, 1500);
}

void bottomBun() {

  // move to station and home horizontal slide
  addWaitingTask([](){
    follow_main_line(0.6, 0.15);
    initVerticalSlides(); 
  }, [](){return atStation(RIGHT, 1);});

  // align wheels with station and home vertical slides
  addWaitingTask([](){
    follow_main_line(0.5, 0.15);
    initHorizontalSlide();
    moveSpatula(10);
  }, [](){return drive_for_set_distance(150);});
  addTimedTask([](){stop_following_main_line();}, 50); 
  
  addWaitingTask([](){
    swingTurnLeft(130, 0.65);
    pryFry(0, 1);
  }, [](){return drive_to_integral_cut();});

  addTimedTask([](){moveSpatula(-10);}, 250);

  addWaitingTimedTask([](){activePositionVerticalSlides(28);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // lower onto bun
  addWaitingTimedTask([](){activePositionHorizontalSlide(130);}, [](){return horizontal_slide_to_integral_cut();}, 2500); // pull in bun
  
  // if it struggles to pull in bun, it tries again
  addWaitingTimedTask([](){
    activePositionVerticalSlides(45);
    activePositionHorizontalSlide(230);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME, true);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(29);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME, true);
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(155);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME, true);

  // back up to leave station
  addWaitingTimedTask([](){
    driveStraightToPosition(-10, ENCODER_FORWARD_POWER);
    activePositionVerticalSlides(65);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // raise spatula and vertical slides to keep bun in
  addTimedTask([](){moveSpatula(15);}, 100);

  // turn onto main line
  addWaitingTask([](){swingTurnRight(115, 0.8);}, [](){return drive_to_integral_cut();});
  addWaitingTask([](){swingTurnRight(200, 0.64);}, [](){return onTape(FRONT_RIGHT_SENSOR);});
  addTimedTask([](){relaxDrivetrain();}, 50);

  // go to plates to drop off bun
  addWaitingTask([](){follow_main_line(0.60, 0.2);}, [](){return atStation(LEFT, 1);});
  addWaitingTask([](){follow_main_line(0.5, 0.15);}, [](){return drive_for_set_distance(20);});
  addTimedTask([](){stop_following_main_line();}, 50); 
  addWaitingTask([](){swingTurnRight(140, 0.75);}, [](){return drive_to_integral_cut();});

  // move arm behind bun to push it off
  addWaitingTimedTask([](){
    activePositionVerticalSlides(90);
    moveSpatula(0);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // push off bun
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(3);
    pryFry(FRY_PRY_MIN_ANGLE, 4.0);
  }, [](){return horizontal_slide_to_integral_cut();},STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(42);
    activePositionHorizontalSlide(180);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // turn back onto line
  addWaitingTask([](){
    sendRobotMessage(BOTTOM_BUN_READY);
    swingTurnRight(-145, 0.75);
    activePositionHorizontalSlide(50);
    activePositionVerticalSlides(90);
  }, [](){return drive_to_integral_cut();});

  // turn 180 degrees
  addWaitingTask([](){
    moveSpatula(15);
    turnLeftOnSpot(NINTY_DEGREE_ON_SPOT_TURN * 2 - 25, 0.75);
  }, [](){return drive_to_integral_cut();});
  addWaitingTask([](){turnLeftOnSpot(200, 0.65);}, [](){return onTape(FRONT_LEFT_SENSOR);});
  addTimedTask([](){relaxDrivetrain();}, 50);
}

void patty() {
 // move to station
  addWaitingTask([](){
    follow_main_line(0.725, 0.2);
    pryFry(0, 1);
  }, [](){return atStation(RIGHT, 1);});
  
  addWaitingTask([](){
    follow_main_line(0.65, 0.175);
  }, [](){return atStation(LEFT, 1);});

  // align wheels with station
  addWaitingTask([](){
    follow_main_line(0.5, 0.15);
    activePositionHorizontalSlide(245);
    moveSpatula(10);
  }, [](){return drive_for_set_distance(150);});
  addTimedTask([](){stop_following_main_line();}, 50); 

  addWaitingTask([](){
    swingTurnRight(130, 0.65);
  }, [](){return drive_to_integral_cut();});

  addTimedTask([](){
    moveSpatula(-10);
  }, 100);

  addWaitingTimedTask([](){activePositionVerticalSlides(23);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // lower onto patty
  addWaitingTimedTask([](){activePositionHorizontalSlide(140);}, [](){return horizontal_slide_to_integral_cut();}, 1250); // pull in patty

  // if it struggles to pull in patty, it tries again
  addWaitingTask([](){
    activePositionVerticalSlides(45);
    activePositionHorizontalSlide(230);
  }, [](){return horizontal_slide_to_integral_cut();}, true);
  addWaitingTask([](){
    activePositionVerticalSlides(25);
  }, [](){return vertical_slide_to_integral_cut();}, true);
  addWaitingTask([](){
    activePositionHorizontalSlide(140);
  }, [](){return horizontal_slide_to_integral_cut();}, true);

  // back up to leave station
  addWaitingTimedTask([](){
    driveStraightToPosition(-10, 0.6);
    activePositionVerticalSlides(65);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // raise spatula and vertical slides to keep bun in
  addTimedTask([](){moveSpatula(15);}, 100);

  // turn onto main line
  addWaitingTask([](){swingTurnLeft(-175, 0.9);}, [](){return drive_to_integral_cut();});
  addWaitingTask([](){swingTurnRight(200, 0.64);}, [](){return onTape(FRONT_RIGHT_SENSOR);});
  addTimedTask([](){relaxDrivetrain();}, 50);

  // go to cooktop to drop off patty
  addWaitingTask([](){follow_main_line(0.6, 0.15);}, [](){return atStation(LEFT, 1);});
  addWaitingTask([](){follow_main_line(0.6, 0.15);}, [](){return drive_for_set_distance(185);});
  addTimedTask([](){stop_following_main_line();}, 50); 
  addWaitingTask([](){swingTurnRight(120, 0.75);}, [](){return drive_to_integral_cut();});

  // move arm behind patty to push it off
  addWaitingTimedTask([](){
    activePositionVerticalSlides(90);
    moveSpatula(0);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // push off patty
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(2);
    pryFry(FRY_PRY_MIN_ANGLE, 4.0);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(40);
    activePositionHorizontalSlide(230);
    sendRobotMessage(PATTY_READY);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  addWaitingTask([](){
    swingTurnRight(-115, 0.75);
  }, [](){return drive_to_integral_cut();});
  addWaitingTask([](){
    swingTurnRight(-250, 0.64); 
    activePositionVerticalSlides(90);
  }, [](){return onTape(FRONT_LEFT_SENSOR);});
}

void fry() {

  addWaitingTask([](){
    follow_main_line(0.6, 0.15);
  }, [](){return atStation(LEFT, 1);});

  addWaitingTask([](){
    follow_main_line(0.5, 0.15);
    moveSpatula(10);
  }, [](){return drive_for_set_distance(60);});

  // turn around
  addWaitingTask([](){
    turnLeftOnSpot(60, 0.65);
    activePositionHorizontalSlide(235);
  }, [](){return drive_to_integral_cut();});
  addWaitingTask([](){
    swingTurnRight(-94, 0.63);
  }, [](){return onTape(FRONT_LEFT_SENSOR);});
  addWaitingTask([](){swingTurnRight(1, 0.63);}, [](){return drive_to_integral_cut();});
  addTimedTask([](){relaxDrivetrain();}, 50);

  addWaitingTask([](){driveStraightToPosition(-50, 0.55);}, [](){return drive_to_integral_cut();});

  addWaitingTask([](){
    follow_main_line(0.44 , 0.1);
  }, [](){return onTape(CENTER_SENSOR);});

  addTimedTask([](){
    stop_following_main_line();
    moveSpatula(-10);
  }, 100);

  addWaitingTimedTask([](){activePositionHorizontalSlide(183);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);  // reach for fries
  addWaitingTimedTask([](){activePositionVerticalSlides(42);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // make fry guide right height to guide
  addTimedTask([](){pryFry(0, 0.4);}, 1000); // guide fries
  addWaitingTimedTask([](){activePositionVerticalSlides(5);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // chomp down on fries
  addTimedTask([](){pryFry(FRY_PRY_MIN_ANGLE, 0.8);}, 1000); // pry fries
  addWaitingTimedTask([](){activePositionVerticalSlides(68);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // go above fries
  addWaitingTimedTask([](){activePositionHorizontalSlide(217);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // get fry guide behind pried fry
  addWaitingTimedTask([](){activePositionVerticalSlides(38);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // make fry guide correct height to pull in
  addWaitingTimedTask([](){activePositionHorizontalSlide(108);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // pull fry into spatula
  addWaitingTimedTask([](){activePositionVerticalSlides(68);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){activePositionHorizontalSlide(50);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // pull fry into spatula

  // raise spatula and vertical slides to keep fry in
  addWaitingTimedTask([](){
    activePositionVerticalSlides(42);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  addWaitingTask([](){
    moveSpatula(10);
    driveStraightToPosition(-10, 0.6);
  }, [](){return drive_to_integral_cut();});

  // go to cooktop to drop off fry
  addWaitingTask([](){
    turnLeftOnSpot(85, 0.65);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){
    swingTurnLeft(100, 0.65);
  }, [](){return drive_to_integral_cut();});

  addTimedTask([](){relaxDrivetrain();}, 50);

  addWaitingTimedTask([](){
    activePositionVerticalSlides(75);
    moveSpatula(0);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // go above fries
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(2);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(28);
    activePositionHorizontalSlide(200);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // turn around
  addWaitingTask([](){
    sendRobotMessage(FRY_READY);
    moveSpatula(10);
    swingTurnLeft(-85, 0.7);
    activePositionVerticalSlides(90);
    activePositionHorizontalSlide(225);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){swingTurnLeft(-250, 0.64);}, [](){return onTape(FRONT_RIGHT_SENSOR);});
  addTimedTask([](){relaxDrivetrain();}, 50);
}

void topBun() {

  addWaitingTask([](){
    follow_main_line(0.65, 0.2);
    pryFry(0, 1);
  }, [](){return atStation(LEFT, 1);}); 

  addWaitingTask([](){follow_main_line(0.65, 0.15);}, [](){return drive_for_set_distance(160);});

  addWaitingTask([](){swingTurnRight(115, 0.65);}, [](){return drive_to_integral_cut();});
  addTimedTask([](){relaxDrivetrain();}, 50);

  addTimedTask([](){moveSpatula(-10);}, 100);
  addWaitingTimedTask([](){activePositionVerticalSlides(55);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // lower onto bun
  addWaitingTimedTask([](){activePositionHorizontalSlide(125);}, [](){return horizontal_slide_to_integral_cut();}, 2500); // pull in bun
  
  // if it struggles to pull in bun, it tries again
  addWaitingTask([](){
    activePositionVerticalSlides(90);
    activePositionHorizontalSlide(235);
  }, [](){return vertical_slide_to_integral_cut();}, true);
  addWaitingTask([](){
    activePositionVerticalSlides(55);
  }, [](){return vertical_slide_to_integral_cut();}, true);
  addWaitingTask([](){
    activePositionHorizontalSlide(135);
  }, [](){return horizontal_slide_to_integral_cut();}, true);

  // back up to leave station
  addWaitingTimedTask([](){
    activePositionVerticalSlides(80);
    pryFry(FRY_PRY_MIN_ANGLE, 0.5);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // raise spatula and vertical slides to keep bun in
  addWaitingTask([](){
    moveSpatula(20);
    driveStraightToPosition(-35, ENCODER_FORWARD_POWER);
  }, [](){return drive_to_integral_cut();});

  // go to cooktop to drop off bun
  addWaitingTask([](){
    turnLeftOnSpot(168, 0.65);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){
    driveStraightToPosition(60, 0.65);
  }, [](){return drive_to_integral_cut();});

  addTimedTask([](){
    relaxDrivetrain();
    moveSpatula(0);
  }, 100);

  // push off bun
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(3);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(50);
    activePositionHorizontalSlide(245);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  // addWaitingTimedTask([](){activePositionHorizontalSlide(245);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // turn back onto line
  addWaitingTask([](){
    sendRobotMessage(TOP_BUN_READY);
    activePositionVerticalSlides(90);
    activePositionHorizontalSlide(50);
    swingTurnRight(-110, 0.75);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){swingTurnRight(-200, 0.64);}, [](){return onTape(FRONT_LEFT_SENSOR);});
}

void secondFry() {
  addWaitingTask([](){
    follow_main_line(0.6, 0.15);
  }, [](){return atStation(LEFT, 1);});

  addWaitingTask([](){
    follow_main_line(0.5, 0.15);
    moveSpatula(10);
  }, [](){return drive_for_set_distance(60);});

  // turns into station
  addWaitingTask([](){
    turnLeftOnSpot(60, 0.65);
    activePositionHorizontalSlide(235);
  }, [](){return drive_to_integral_cut();});
  addWaitingTask([](){
    swingTurnRight(-94, 0.63);
  }, [](){return onTape(FRONT_LEFT_SENSOR);});
  addWaitingTask([](){swingTurnRight(1, 0.63);}, [](){return drive_to_integral_cut();});
  addTimedTask([](){relaxDrivetrain();}, 50);

  addWaitingTask([](){driveStraightToPosition(-50, 0.55);}, [](){return drive_to_integral_cut();});

  addWaitingTask([](){
    follow_main_line(0.44 , 0.1);
  }, [](){return onTape(CENTER_SENSOR);});

  addTimedTask([](){
    stop_following_main_line();
    moveSpatula(-10);
  }, 100);

  addWaitingTimedTask([](){activePositionHorizontalSlide(210);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);  // reach for fries
  addWaitingTimedTask([](){activePositionVerticalSlides(42);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // make fry guide right height to guide
  addTimedTask([](){pryFry(0, 0.4);}, 1000); // guide fries
  addWaitingTimedTask([](){activePositionVerticalSlides(5);}, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // chomp down on fries
  addWaitingTimedTask([](){activePositionHorizontalSlide(183);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addTimedTask([](){pryFry(FRY_PRY_MIN_ANGLE, 0.8);}, 1000); // pry fries
  addWaitingTimedTask([](){activePositionVerticalSlides(68);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // go above fries
  addWaitingTimedTask([](){activePositionHorizontalSlide(217);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // get fry guide behind pried fry
  addWaitingTimedTask([](){activePositionVerticalSlides(38);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // make fry guide correct height to pull in
  addWaitingTimedTask([](){activePositionHorizontalSlide(108);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // pull fry into spatula
  addWaitingTimedTask([](){activePositionVerticalSlides(68);},  [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){activePositionHorizontalSlide(50);}, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // pull fry into spatula

  // raise spatula and vertical slides to keep fry in
  addWaitingTimedTask([](){
    activePositionVerticalSlides(42);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  addWaitingTask([](){
    moveSpatula(10);
    driveStraightToPosition(-10, 0.6);
  }, [](){return drive_to_integral_cut();});

  // go to cooktop to drop off fry
  addWaitingTask([](){
    turnLeftOnSpot(85, 0.65);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){
    swingTurnLeft(100, 0.65);
  }, [](){return drive_to_integral_cut();});

  addTimedTask([](){relaxDrivetrain();}, 50);

  addWaitingTimedTask([](){
    moveSpatula(0);
    activePositionVerticalSlides(75);
  }, [](){return vertical_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME); // go above fries
  addWaitingTimedTask([](){
    activePositionHorizontalSlide(2);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);
  addWaitingTimedTask([](){
    activePositionVerticalSlides(32);
    activePositionHorizontalSlide(200);
  }, [](){return horizontal_slide_to_integral_cut();}, STANDARD_TIME_OUT_TIME);

  // turn around
  addWaitingTask([](){
    sendRobotMessage(FRY_2_READY);
    moveSpatula(10);
    swingTurnLeft(-85, 0.7);
    activePositionVerticalSlides(90);
    activePositionHorizontalSlide(225);
  }, [](){return drive_to_integral_cut();});

  addWaitingTask([](){swingTurnLeft(-250, 0.64);}, [](){return onTape(FRONT_RIGHT_SENSOR);});
  addTimedTask([](){relaxDrivetrain();}, 50);
}

bool start;

long currentTime;
long lastTime;
int tickRate = 5;

void setup() {
  Serial.begin(115200);

  #ifndef WIFI_DISABLED
  initConfigServer();
  #endif

  // initOTA();
  pinMode(START_BUTTON, INPUT_PULLUP);
  while (digitalRead(START_BUTTON) == HIGH) {}

  // initialise everything
  initRgbLed();
  init_motor_control();
  init_tape_following(TAPE_SENSOR_THRESHOLD);
  initEncodedDrivetrain();
  initRobotMessenger(false);

  bottomBun();
  patty();
  fry();
  topBun();
  secondFry();

  lastTime = millis();
  delay(20);
}

void loop() {
  currentTime = millis();
  if (currentTime - lastTime >= tickRate) {
    tickTaskSequence();
    tickDrivetrain();
    tickHorizontalSlide();
    tickVerticalSlides();
    tickFryPry();
    tickTapeFollowing();
    // ArduinoOTA.handle();
    lastTime = currentTime;
  }
}