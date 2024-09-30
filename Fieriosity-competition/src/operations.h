#include "motor_control.h"
#include "encodedDrivetrain.h"
#include "taskSequence.h"
#define TURN_SPEED 0.65
#define TAPE_FOLLOW_SPEED 1.0
#define ESP_NOW_WAIT_MS 15000
#define BOTTOM_BUN_WAIT_MILLIS 10000
#define PATTY_WAIT_MILLIS 10000
#define FRY_WAIT_MILLIS 10000
#define TOP_BUN_WAIT_MILLIS 10000
#define FRY_2_WAIT_MILLIS 10000
// drives to a station while prepping the arm
#define drive_to_relative_station(sideToCountTapes, tapesToCount, encoderTicks)\
  addWaitingTask([](){\
    follow_main_line(TAPE_FOLLOW_SPEED, 0.2, encoderTicks);\
    activePositionArm(-0.6);\
    extendArm(0.6);\
  }, [](){return atStation(sideToCountTapes, tapesToCount);});

#define serve(servingSide)\
  addWaitingTask([](){\
    stop_following_main_line(); \
    driveStraightToPosition(20, 0.6);\
    relaxBed();\
    activePositionArm(-0.25);\
    retractArm(0.9);\
  }, isArmExtensionIdle);\
  addTimedTask([](){activePositionBed(0.55);}, 1500);\
  if (servingSide == RIGHT) {\
    addWaitingTimedTask([](){swingTurnLeft(-148, 0.75);}, isDrivetrainHappy, 5000);\
  } else if (servingSide == LEFT) {\
    addWaitingTimedTask([](){swingTurnRight(-148, 0.75);}, isDrivetrainHappy, 5000);\
  }\
  addWaitingTimedTask([](){driveStraightToPosition(35, 0.6);}, isDrivetrainHappy, 2000);\
  addTimedTask([](){\
    relaxDrivetrain();\
    relaxBed(); /* let it rest on the counter */\
    run_bed(0.45);\
    activePositionArm(-0.12);\
    }, 1000);\
  addWaitingTask([](){extendArm(1.0);}, isArmExtensionIdle);\


// Call this when the appropriate line is detected
// Drives into the station with a set distance, positioning the bed wherever it should be
// Does not handle positioning the arm
#define position_in_station(forwardAmount, turnDirection, turnAmount, bedPosition, distanceIntoStation)\
  addWaitingTimedTask([](){\
    stop_following_main_line();\
    driveStraightToPosition(forwardAmount, 0);\
  }, isDrivetrainHappy, 900);\
  if (turnDirection == LEFT) {\
    addWaitingTimedTask([](){\
      swingTurnLeft(turnAmount, TURN_SPEED);\
    }, isDrivetrainHappy, 5000);\
  } else {\
    addWaitingTimedTask([](){\
      swingTurnRight(turnAmount, TURN_SPEED);\
    }, isDrivetrainHappy, 5000);\
  }\
  addTimedTask([](){\
    activePositionBed(bedPosition);\
  }, 0);\
  addWaitingTimedTask([](){\
    driveStraightToPosition(distanceIntoStation, 0.5);\
  }, isDrivetrainHappy, 1500);\

// lowers the arm and retracts the rake
#define sweep_item(armPosition, retractSpeed, retractionPosition)\
  addTimedTask([](){\
    relaxDrivetrain(); /*Needed to stably lower the arm*/\
    activePositionArm(-0.1);\
  }, 500);\
  addTimedTask([](){\
    relaxArmRotation();\
  }, 0);\
  addWaitingTask([](){positionExtension(retractSpeed, retractionPosition);}, isArmExtensionIdle);

// moves from the station to the main line
#define exit_station(distanceIntoStation, endingTurnDirection)\
  addTimedTask([](){\
    relaxBed();\
    driveStraightToPosition(-(distanceIntoStation), 0.5);\
  }, 200);\
  addWaitingTimedTask([](){\
  }, isDrivetrainHappy, 700);\
  if (endingTurnDirection == LEFT) {\
    addWaitingTimedTask([](){\
      swingTurnRight(-200, TURN_SPEED);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});\
    /*addTimedTask([](){relaxDrivetrain();}, 100);\
    addWaitingTask([](){\
      swingTurnRight(200, 0.6);\
    }, [](){return onTape(FRONT_RIGHT_SENSOR);});*/\
  }\
  else {\
    addWaitingTimedTask([](){\
      swingTurnLeft(-200, TURN_SPEED);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_RIGHT_SENSOR);});\
    /*addTimedTask([](){swingTurnLeft(-10, 0.6);}, 100);\
    addWaitingTask([](){\
      swingTurnLeft(200, 0.6);\
    }, [](){return onTape(FRONT_LEFT_SENSOR);});*/\
  }\

  #define exit_serving(endingTurnDirection)\
  addWaitingTimedTask([](){\
    relaxBed();\
    driveStraightToPosition(-(35), 0.5);\
    retractArm(0.6);\
  }, isDrivetrainHappy, 900);\
  if (endingTurnDirection == LEFT) {\
    addWaitingTimedTask([](){\
      swingTurnLeft(200, 0.6);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});\
    /*addTimedTask([](){relaxDrivetrain();}, 100);\
    addWaitingTask([](){\
      swingTurnRight(200, 0.6);\
    }, [](){return onTape(FRONT_RIGHT_SENSOR);});*/\
  }\
  else {\
    addWaitingTimedTask([](){\
      swingTurnRight(200, 0.6);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_RIGHT_SENSOR);});\
    /*addTimedTask([](){swingTurnLeft(-10, 0.6);}, 100);\
    addWaitingTask([](){\
      swingTurnLeft(200, 0.6);\
    }, [](){return onTape(FRONT_LEFT_SENSOR);});*/\
  }\
  

// Call this when the appropriate line is detected
// In this multi-line function-like define, the backslashes are "line continuers" and must be the very last character of each line
#define pick_up_at_station(forwardAmount, TurnDirection, TurnAmount, bedPosition, endingTurnDirection, distanceIntoStation, retractDistance)\
  position_in_station(forwardAmount, TurnDirection, TurnAmount, bedPosition, distanceIntoStation);\
  sweep_item(0.0, 0.9, retractDistance);\
  exit_station(distanceIntoStation, endingTurnDirection);

#define pick_up_at_station_fries(forwardAmount, TurnDirection, TurnAmount, bedPosition, endingTurnDirection, distanceIntoStation, retractDistance)\
  position_in_station(forwardAmount, TurnDirection, TurnAmount, bedPosition, distanceIntoStation);\
  addWaitingTimedTask([](){}, [](){return getCurrentPhase() == FRY_2_READY && millis() > getPhaseReadchedMillis(FRY_2_READY) + 5000;}, FRY_2_WAIT_MILLIS);\
  sweep_item(0.0, 0.9, retractDistance);\
  exit_station(distanceIntoStation, endingTurnDirection);

#define _180_pick_up(forwardAmount, firstTurnDirection, firstTurnAmount, firstBedPosition, firstDistanceIntoStation, firstRetractDistance, secondTurnAmount, secondBedPosition, secondDistanceIntoStation, secondRetractDistance)\
  position_in_station(forwardAmount, firstTurnDirection, firstTurnAmount, firstBedPosition, firstDistanceIntoStation);\
  sweep_item(0.0, 0.9, firstRetractDistance);\
  addWaitingTimedTask([](){\
    relaxBed();\
    driveStraightToPosition(-(firstDistanceIntoStation), 0.5);\
  }, isDrivetrainHappy, 900);\
  if (firstTurnDirection == LEFT) {\
    addWaitingTimedTask([](){\
      swingTurnRight(-200, TURN_SPEED);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_LEFT_SENSOR);});\
    /*addTimedTask([](){relaxDrivetrain();}, 100);\
    addWaitingTask([](){\
      swingTurnRight(200, 0.6);\
    }, [](){return onTape(FRONT_RIGHT_SENSOR);});*/\
    addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6);}, 0);\
    addWaitingTimedTask([](){ extendArm(0.7); swingTurnLeft(secondTurnAmount, TURN_SPEED);}, isDrivetrainHappy, 5000);\
    addWaitingTimedTask([](){driveStraightToPosition(secondDistanceIntoStation, 0.5); activePositionBed(secondBedPosition);}, isDrivetrainHappy, 1500);\
    sweep_item(-0.05, 0.9, secondRetractDistance);\
    exit_station(secondDistanceIntoStation, RIGHT);\
  }\
  else {\
    addWaitingTimedTask([](){\
      swingTurnLeft(-200, 0.6);\
    }, isDrivetrainHappy, 900);\
    addWaitingTask([](){}, [](){return onTape(FRONT_RIGHT_SENSOR);});\
    /*addTimedTask([](){swingTurnLeft(-10, 0.6);}, 100);\
    addWaitingTask([](){\
      swingTurnLeft(200, 0.6);\
    }, [](){return onTape(FRONT_LEFT_SENSOR);});*/\
    addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6);}, 0);\
    addWaitingTimedTask([](){ extendArm(0.7); swingTurnRight(secondTurnAmount, TURN_SPEED);}, isDrivetrainHappy, 5000);\
    addWaitingTimedTask([](){driveStraightToPosition(secondDistanceIntoStation, 0.5); activePositionBed(secondBedPosition);}, isDrivetrainHappy, 1500);\
    sweep_item(-0.05, 0.9, secondRetractDistance);\
    exit_station(secondDistanceIntoStation, LEFT);\
  }

/*#define pick_up_and_wait(turnInDirection, turnAmount, firstBedPosition, firstDistanceIntoStation, secondBedPosition, secondDistanceIntoStation, waitMillis, turnOutDirection)\
  position_in_station(turnInDirection, turnAmount, firstBedPosition, firstDistanceIntoStation);\
  sweep_item(-0.1, 0.9);\
  addWaitingTimedTask([](){driveStraightToPosition(secondDistanceIntoStation - firstDistanceIntoStation, 0.5); relaxBed();}, isDrivetrainHappy, 2000);\
  addTimedTask([](){activePositionBed(secondBedPosition); relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);\
  addTimedTask([](){}, waitMillis);\
  sweep_item(-0.1, 0.9);\
  exit_station(secondDistanceIntoStation, turnOutDirection);*/

#define smart_patty_pick_up(forwardAmount, TurnDirection, TurnAmount, bedPosition, endingTurnDirection, distanceIntoStation, retractDistance)\
  position_in_station(forwardAmount, TurnDirection, TurnAmount, bedPosition, distanceIntoStation);\
  addWaitingTimedTask([](){}, [](){return getCurrentPhase() == PATTY_READY && millis() > getPhaseReadchedMillis(PATTY_READY) + 5000;}, PATTY_WAIT_MILLIS);\
  sweep_item(0.0, 0.9, retractDistance);\
  exit_station(distanceIntoStation, endingTurnDirection);

#define smart_top_bun_pick_up(forwardAmount, TurnDirection, TurnAmount, bedPosition, endingTurnDirection, distanceIntoStation, topBunArmRetractPosition)\
  position_in_station(forwardAmount, TurnDirection, TurnAmount, bedPosition, distanceIntoStation);\
  addTimedTask([](){\
    relaxDrivetrain();\
    activePositionArm(-0.20);\
  }, 1500);\
  addWaitingTask([](){retractArm(0.9);}, [](){return getExtensionPosition() > topBunArmRetractPosition;});\
  addTimedTask([](){stopArmExtension();}, 1000);\
  exit_station(distanceIntoStation, endingTurnDirection);

#define smart_plate_and_bottom_bun_pick_up(forwardAmount, turnAmount, firstBedPosition, firstDistanceIntoStation, secondBedPosition, secondDistanceIntoStation, bottomBunArmRetractPosition)\
  position_in_station(forwardAmount, LEFT, turnAmount, firstBedPosition, firstDistanceIntoStation);\
  sweep_item(-0.1, 0.9, 100000);\
  addWaitingTimedTask([](){driveStraightToPosition((secondDistanceIntoStation - firstDistanceIntoStation), 0.5); relaxBed();}, isDrivetrainHappy, 2000);\
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);\
  addWaitingTimedTask([](){}, [](){return getCurrentPhase() >= BOTTOM_BUN_READY && millis() > getPhaseReadchedMillis(BOTTOM_BUN_READY) + 500;}, BOTTOM_BUN_WAIT_MILLIS);\
  addTimedTask([](){activePositionBed(secondBedPosition);}, 0);\
  sweep_item(-0.1, 0.9, bottomBunArmRetractPosition)\
  exit_station(0, LEFT);

#define smart_fries_and_top_bun_pick_up(forwardAmount, turnAmount, firstBedPosition, firstDistanceIntoStation, secondBedPosition, secondDistanceIntoStation, topBunArmRetractPosition)\
  position_in_station(forwardAmount, RIGHT, turnAmount, firstBedPosition, firstDistanceIntoStation);\
  addWaitingTimedTask([](){}, [](){return getCurrentPhase() >= FRY_READY && millis() > getPhaseReadchedMillis(FRY_READY) + 5000;}, FRY_WAIT_MILLIS);\
  sweep_item(-0.1, 0.9, 100000);\
  addWaitingTimedTask([](){driveStraightToPosition((secondDistanceIntoStation - firstDistanceIntoStation), 0.5); relaxBed();}, isDrivetrainHappy, 2000);\
  addTimedTask([](){relaxDrivetrain(); activePositionArm(-0.6); extendArm(0.6);}, 3000);\
  addWaitingTimedTask([](){}, [](){return getCurrentPhase() == TOP_BUN_READY && millis() > getPhaseReadchedMillis(TOP_BUN_READY) + 500;}, TOP_BUN_WAIT_MILLIS);\
  addTimedTask([](){activePositionBed(secondBedPosition); relaxDrivetrain(); activePositionArm(-0.1); }, 500); addTimedTask([](){ relaxArmRotation(); }, 1000);\
  addWaitingTask([](){retractArm(0.9);}, [](){return getExtensionPosition() > topBunArmRetractPosition;});\
  addTimedTask([](){stopArmExtension();}, 1000);\
  exit_station(0, LEFT);
