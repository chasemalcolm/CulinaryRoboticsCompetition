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

#define PIN_SDA 46
#define PIN_SCL 3

// drives to a station while prepping the arm
#define drive_to_relative_station(sideToCountTapes, tapesToCount)\
  addWaitingTask([](){ \
    follow_main_line(0.7, 0.2); \
    activePositionArm(-0.6); \
    extendArm(0.6); \ 
  }, [](){return atStation(sideToCountTapes, tapesToCount);});

#define serve(servingSide)\
  addTimedTask([](){stop_following_main_line();}, 0);\
  addWaitingTask([](){\
  relaxBed();\
    activePositionArm(-0.25);\
    retractArm(0.5);\
  }, isArmExtensionIdle);\
  if (servingSide == RIGHT) {\
  addTimedTask([](){activePositionBed(0.55);}, 3000);\
    addTimedTask([](){\
    swingTurnLeft(-143, 0.6);\
  }, 5000);\
  } else if (servingSide == LEFT) {\
    addTimedTask([](){activePositionBed(0.55);}, 3000);\
    addTimedTask([](){\
    swingTurnRight(-143, 0.6);\
  }, 5000);\
  }\
  addTimedTask([](){driveStraightToPosition(45, 0.6);}, 2000);\
  addTimedTask([](){\
    relaxDrivetrain();\
    relaxBed(); /* let it rest on the counter*/ \
    activePositionArm(-0.12);\
    }, 1000);\
  addWaitingTask([](){extendArm(0.5);}, isArmExtensionIdle);\


// Call this when the appropriate line is detected
// Drives into the station with a set distance, positioning the bed wherever it should be
// Does not handle positioning the arm
#define position_in_station(turnDirection, turnAmount, bedPosition, distanceIntoStation)\
  addTimedTask([](){\
    stop_following_main_line();\
    driveStraightToPosition(15, 0.6);\
  }, 900);\
  addTimedTask([](){\
    activePositionBed(bedPosition);\
  }, 0);\
  if (turnDirection == LEFT) {\
    addTimedTask([](){\
      swingTurnLeft(turnAmount, 0.6);\
    }, 2000);\
  } else { \
    addTimedTask([](){\
      swingTurnRight(turnAmount, 0.6);\
    }, 2000);\
  }\
  addTimedTask([](){\
    driveStraightToPosition(distanceIntoStation, 0.5);\
  }, 1500);\

// lowers the arm and retracts the rake
#define sweep_item(armPosition, retractSpeed) \
  addTimedTask([](){\
    relaxDrivetrain(); /*Needed to stably lower the arm*/ \
    activePositionArm(armPosition);\
  }, 750);\
  addTimedTask([](){\
    relaxArmRotation();\
  }, 750);\
  addWaitingTask([](){retractArm(retractSpeed);}, isArmExtensionIdle);\
  relaxArmRotation();

// moves from the station to the main line
#define exit_station(distanceIntoStation, endingTurnDirection)\
  addTimedTask([](){\
    relaxBed();\
    driveStraightToPosition(-(distanceIntoStation), 0.5);\
  }, 900);\
  if (endingTurnDirection == LEFT) {\
    addTimedTask([](){\
      swingTurnRight(-200, 0.6);\
    }, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});\
    /*addTimedTask([](){relaxDrivetrain();}, 100);\
    addWaitingTask([](){\
      swingTurnRight(200, 0.6);\
    }, [](){return onTape(FRONT_RIGHT_SENSOR);});*/\
  }\
  else {\
    addTimedTask([](){\
      swingTurnLeft(-200, 0.6);\
    }, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_RIGHT_SENSOR);});\
    /*addTimedTask([](){swingTurnLeft(-10, 0.6);}, 100);\
    addWaitingTask([](){\
      swingTurnLeft(200, 0.6);\
    }, [](){return onTape(FRONT_LEFT_SENSOR);});*/\
  }\
  

// Call this when the appropriate line is detected
// In this multi-line function-like define, the backslashes are "line continuers" and must be the very last character of each line
#define pick_up_at_station(TurnDirection, TurnAmount, bedPosition, endingTurnDirection) \
  position_in_station(TurnDirection, TurnAmount, bedPosition, 30);\
  sweep_item(0.0, 0.9);\
  exit_station(30, endingTurnDirection);

void go(double distance) {
}

void oneSalad() {
  // drive to plate
  drive_to_relative_station(LEFT, 3);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.0, LEFT);

  // pickup lettuce
  addTimedTask([](){activePositionBed(0.0);}, 0);
  addTimedTask([](){activePositionArm(-0.6); extendArm(0.7); swingTurnLeft(143, 0.6);}, 2000);
  addTimedTask([](){driveStraightToPosition(30, 0.5);}, 1500);
  sweep_item(-0.1, 0.9);
  exit_station(30, RIGHT);
  
  // drive to tomato
  drive_to_relative_station(RIGHT, 3);
  
  // pickup tomato
  pick_up_at_station(RIGHT, 136, 0.2, RIGHT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 2000);

  // serve!
  serve(RIGHT);
}

void saladAndFries() {
// SALAD 

  // drive to plate
  drive_to_relative_station(LEFT, 3);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.05, LEFT);

  // pickup lettuce
  addTimedTask([](){relaxDrivetrain();activePositionBed(0.0); activePositionArm(-0.6); }, 2000);
  addTimedTask([](){extendArm(0.7); swingTurnLeft(143, 0.6);}, 3000);
  addTimedTask([](){driveStraightToPosition(30, 0.5);}, 1500);
  sweep_item(-0.0, 0.9);
  exit_station(30, RIGHT);
  
  // drive to tomato
  drive_to_relative_station(RIGHT, 3);
  
  // pickup tomato
  pick_up_at_station(RIGHT, 136, 0.2, RIGHT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 2000);

  // serve!
  serve(RIGHT);

// FRY

  // get out of serving station
  addTimedTask([](){relaxBed(); relaxArmRotation(), driveStraightToPosition(-45, 0.5);}, 3000);
  addTimedTask([](){swingTurnLeft(200, 0.7);}, 1000);
  addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});
  addTimedTask([](){activePosition(0, 0);}, 1000);

  // drive to plate
  drive_to_relative_station(LEFT, 2);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.05, LEFT);

  // drive to cooktop
  drive_to_relative_station(RIGHT, 1);

  // pickup fry
  pick_up_at_station(RIGHT, 136, 0.05, LEFT);
  
  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 1000);

  // serve!
  serve(LEFT);
}

void twoSalad() {
// SALAD 1

  // drive to plate
  drive_to_relative_station(LEFT, 3);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.05, LEFT);

  // pickup lettuce
  addTimedTask([](){relaxDrivetrain(); activePositionBed(0.0); activePositionArm(-0.6); }, 2000);
  addTimedTask([](){extendArm(0.7); swingTurnLeft(143, 0.6);}, 2000);
  addTimedTask([](){driveStraightToPosition(30, 0.5);}, 1500);
  sweep_item(-0.0, 0.9);
  exit_station(30, RIGHT);
  
  // drive to tomato
  drive_to_relative_station(RIGHT, 3);
  
  // pickup tomato
  pick_up_at_station(RIGHT, 136, 0.2, RIGHT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 2000);

  // serve!
  serve(RIGHT);


// SALAD 2

  // get out of serving station
  addTimedTask([](){relaxBed(); relaxArmRotation(), driveStraightToPosition(-45, 0.5);}, 3000);
  addTimedTask([](){swingTurnLeft(200, 0.7);}, 1000);
  addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});
  addTimedTask([](){activePosition(0, 0);}, 1000);

  // drive to plate
  drive_to_relative_station(LEFT, 2);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.05, LEFT);

  // pickup lettuce
  addTimedTask([](){relaxDrivetrain(); activePositionBed(0.0); activePositionArm(-0.6);}, 2000);
  addTimedTask([](){extendArm(0.7); swingTurnLeft(143, 0.6);}, 2000);
  addTimedTask([](){driveStraightToPosition(30, 0.5);}, 1500);
  sweep_item(-0.1, 0.9);
  exit_station(30, RIGHT);
  
  // drive to tomato
  drive_to_relative_station(RIGHT, 3);
  
  // pickup tomato
  pick_up_at_station(RIGHT, 136, 0.2, RIGHT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 2000);

  // serve!
  serve(RIGHT);
}

void burger() {
  // drive to plates
  drive_to_relative_station(LEFT, 3);

  // pickup plates
  position_in_station(LEFT, 136, 0.0, 30);
  sweep_item(-0.1, 0.9);

  // pickup bottom bun
  addTimedTask([](){driveStraightToPosition(-40, 0.5); activePositionBed(0.3);}, 2000); 
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);
  sweep_item(-0.1, 0.9); 
  exit_station(-10, LEFT);

  // drive to cooktop
  drive_to_relative_station(RIGHT, 1);

  // pickup patty
  position_in_station(RIGHT, 136, 0.0, 30);
  sweep_item(-0.1, 0.9);

  // pickup top bun
  addTimedTask([](){driveStraightToPosition(-40, 0.5); activePositionBed(0.3);}, 2000); 
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);
  sweep_item(-0.1, 0.9); 
  exit_station(-10, LEFT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 1000);

  // serve!
  serve(LEFT);
}

void frenchFry() {
  // SALAD 1

  // drive to plate
  drive_to_relative_station(LEFT, 3);
  
  // pickup plate
  pick_up_at_station(LEFT, 136, 0.05, LEFT);

  drive_to_relative_station(RIGHT, 1);

  pick_up_at_station(RIGHT, 140, 0.15, LEFT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 1000);

  // serve!
  serve(LEFT); 


}

void deluxeCheeseBurger() {
  // drive to plates
  drive_to_relative_station(LEFT, 3);

  // pickup plates
  position_in_station(LEFT, 136, 0.0, 30);
  sweep_item(-0.1, 0.9);

  // pickup bottom bun
  addTimedTask([](){driveStraightToPosition(0, 0.5); activePositionBed(0.0);}, 2000); 
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);
  addTimedTask([](){}, 5000);
  sweep_item(-0.1, 0.9); 
  exit_station(30, LEFT);
  
  // drive to cheese
  drive_to_relative_station(LEFT, 1);

  // pickup cheese
  pick_up_at_station(LEFT, 136, 0.2, LEFT);

  // pickup tomato
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); activePositionBed(0.2);}, 1500);
  addTimedTask([](){ extendArm(0.7); swingTurnLeft(143, 0.6);}, 2000);
  addTimedTask([](){driveStraightToPosition(20, 0.5);}, 1500);
  sweep_item(-0.05, 0.9);
  exit_station(20, RIGHT);

  
  // drive to cooktop
  drive_to_relative_station(LEFT, 2);

  
  // pickup patty
  pick_up_at_station(LEFT, 143, 0.2, RIGHT);
  

  // drive to lettuce
  drive_to_relative_station(RIGHT, 1);

  //pickup lettuce
  pick_up_at_station(RIGHT, 143, 0.2, RIGHT);
  
  // drive to cooktop
  drive_to_relative_station(RIGHT, 1);

  //pickup top bun
  pick_up_at_station(RIGHT, 143, 0.2, LEFT);

  // drive to serving station
  addTimedTask([](){follow_main_line(0.7, 0.075);}, 1000);

  // serve!
  serve(LEFT); 
}



void cheesePlate() {
  // pickup plate
  drive_to_relative_station(LEFT, 3);
  pick_up_at_station(LEFT, 136, 0.0, LEFT);
  drive_to_relative_station(LEFT, 1);
  pick_up_at_station(LEFT, 136, 0.1, LEFT);
  
  addTimedTask([](){follow_main_line(0.5, 0.1);}, 4000);

  serve(RIGHT);
  
}


void setup() {
  Serial.begin(115200);

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


  addTweakableVariable("Follow Main Line (0.1 turning)", [](double val){follow_main_line(val, 0.1);}, 0);
  addTweakableVariable("Stop Follow Main Line", [](double val){stop_following_main_line();}, 0);





  addValueReadout("millis", [](){return (double) millis();});
  addValueReadout("Front left", [](){return (double) analogRead(6);});
  addValueReadout("Front right", [](){return (double) analogRead(8);});
  addValueReadout("Left", [](){return (double) analogRead(4);});
  addValueReadout("Right", [](){return (double) analogRead(7);});
  addValueReadout("Center", [](){return (double) analogRead(5);});

  initConfigServer();

  initOTA();

  
  initArm(); // must comebefore the bed
  initBed();

  init_tape_following(3000);

  
  // start button
  addWaitingTask([](){}, [](){return !digitalRead(0);});
  //burger();
  //cheesePlate();
  // serve_from_left();
  // deluxeCheeseBurger();
  saladAndFries();

}



void loop() {
  /*Serial.printf("Left: %d, Right: %d ", LeftEncoder.position, RightEncoder.position);
  Serial.printf("A: %d, B: %d ", digitalRead(ENCODER_RIGHT_PIN_A), digitalRead(ENCODER_RIGHT_PIN_B));
  Serial.printf("\n");*/
  tickDrivetrain();
  tickBed();
  tickArm();
  tickTapeFollowing();

  tickTaskSequence();

  ArduinoOTA.handle();
  Serial.printf("%d \n", millis());
  delay(20); // putting the delay after the print seems to make the vex encoders initialize more reliably.



}
