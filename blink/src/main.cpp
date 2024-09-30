#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "tape_following.h"
#include "motor_control.h"
#include "encodedDrivetrain.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

#define FRONT_LEFT_SENSOR 6
#define FRONT_RIGHT_SENSOR 8
#define LEFT_SENSOR 7
#define RIGHT_SENSOR 4
#define CENTER_SENSOR 5

int countLeft;
int countRight;
bool canCountLeft;
bool canCountRight;
bool following_main_line;
bool stop;
bool do_delay;

long current_millis;
long start_millis;


// ROUTE VARIABLES
volatile int step;
volatile int tapeCount;
volatile bool canCount;
volatile bool sideLeft;
enum Location {Plates, Tomato, Cheese, Cooktop, Lettuce, Cooktop, ServingArea};
enum Action {PickupPlateAndBottomBun, PickupTomato, PickupCheese, PickupPatty, PickupLettuce, PickupTopBunAndFries, ServePlate, PickupPlate, PickupBottomBun, PickupFries, PickupTopBun};

// Change these if changing the order
//enum Location locations[] = {Plates, Tomato, Cheese, Cooktop, Lettuce, Cooktop, ServingArea};
//enum Action actions[] = {PickupPlateAndBottomBun, PickupTomato, PickupCheese, PickupPatty, PickupLettuce, PickupTopBunAndFries, ServePlate};
//int rotationToLocation[] = {90, -90, 180, 90, -90, -90, 90};
//int rotationToMainLine[] = {90, -90, 90, -90, -90, -90, 90};
//int tapeCounts[] = {4, 3, 0, 2, 1, 1, 0};

enum Location location;
enum Action action;

void millisDelay(int delay) {
  long millis1 = millis();
  long millis2 = millis();
  while (millis1 < millis2 + delay) {
    millis1 - millis();
  }
}

bool delaying(long start_time, long current_time, long delay) {
  return current_time < start_time + delay;
}

void countSides() {
  if (!canCountLeft && !onTape(LEFT_SENSOR)) {
    canCountLeft = true;
  }
  if (!canCountRight && !onTape(RIGHT_SENSOR)) {
    canCountRight = true;
  }
  if (canCountLeft && onTape(LEFT_SENSOR)) {
    countLeft++;
    canCountLeft = false;
  }
  if(canCountRight && onTape(RIGHT_SENSOR)) {
    countRight++;
    canCountLeft = false;
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(3, 46);
  init_motor_control();
  init_tape_following(2000);
  initEncodedDrivetrain();

  // ROUTE SETUP
  step = 0;
  tapeCount = 0;
  canCount = true;
  countLeft = 0;
  countRight = 0;
  canCountLeft = false;
  canCountRight = false;
  following_main_line = true;
  stop = false;
  do_delay = true;

  // INTERRUPTS

}

void loop() { 
  current_millis = millis();
  tickDrivetrain();

  if (following_main_line) {
    follow_main_line(0.5, 0.1);
    countSides();
  }

  if (countLeft == 3) {
    following_main_line = false;
    
    if (do_delay) {
      start_millis = millis();
      do_delay = false;
    } else {
      if (delaying(start_millis, current_millis, 1000)) {
        run_drive_motors(0, 0);
      } else {
        swingTurnLeft(143, 0.5);
        countLeft = 0;
        countRight = 0;
      }
    }
  }

/*
  switch (action) {
    // current plan
    case PickupPlateAndBottomBun: {
      pickupPlateAndBottomBun();
    }
    case PickupTomato: {
      pickupTomato();
    }
    case PickupCheese: {
      pickupCheese();
    }
    case PickupPatty: {
      pickupPatty();
    }
    case PickupLettuce: {
      pickupLettuce();
    }
    case PickupTopBunAndFries: {
      pickupTopBunAndFries();
    }
    // extras
    case PickupPlate: {
      pickupPlate();
    }
    case PickupBottomBun: {
      pickupBottomBun();
    }
    case PickupFries: {
      pickupFries();
    }
    case PickupTopBun: {
      pickupTopBun();
    }
  }
  */
}

// FUNCTIONS
// action starts in line with main line, facing the station
// must end stopped on the main line facing the station
// include waiting at location (multiple steps)

void pickupPlateAndBottomBun() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupTopBun() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupTomato() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupCheese() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupPatty() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupLettuce() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupTopBunAndFries() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupFries() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupPlate() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupTopBun() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void pickupBottomBun() {
  // moving forward
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}

void servePlate() {
  // navgating line using beacon
  // rotating and moving to serving area
  // adjusting plate bed location
  // raking plate
  // moving plate bed and arm back
  // moving back
}
