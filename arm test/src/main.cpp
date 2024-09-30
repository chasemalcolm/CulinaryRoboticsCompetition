#include <Arduino.h>
#include <I2CEncoder.h>
#include <Wire.h>
#include <ESP32RotaryEncoder.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LINEAR_MOTOR_A 15
#define LINEAR_MOTOR_B 16

#define RADIAL_MOTOR_A 6
#define RADIAL_MOTOR_B 7

#define BED_MOTOR_A 6
#define BED_MOTOR_B 7

#define CONTRACTED_SWITCH 5
#define EXTENDED_SWITCH 4
//#define RADIAL_HOMING_SWITCH 4

volatile int linear_reading;
volatile int radial_reading;
volatile long start_millis;
volatile long current_millis;
volatile bool extended_button_pressed;
volatile bool contracted_button_pressed;
volatile int encoder_count;

I2CEncoder encoder;

void stopMotors(int MOTOR_A, int MOTOR_B) {
  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
}

void setMotorSpeed(int MOTOR_A, int MOTOR_B, int motor_speed) {
  if (motor_speed > 0) {
    analogWrite(MOTOR_A, motor_speed);
    analogWrite(MOTOR_B, 0);
  } else {
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, -motor_speed);
  }
}

void setRadialPositionUsingPotentiometer(int POTENTIOMETER_PIN, int MOTOR_A, int MOTOR_B, double encoder_higher_threshold, double encoder_lower_threshold, I2CEncoder encoder, int speed) {

  double pot_reading = analogRead(POTENTIOMETER_PIN);
  double position_wanted = encoder_lower_threshold + pot_reading * (encoder_higher_threshold - encoder_lower_threshold) / 4096.0;

  stopMotors(MOTOR_A, MOTOR_B);

  if (position_wanted < encoder.getPosition()) {
    while (position_wanted + 0.1 < encoder.getPosition()) {
      pot_reading = analogRead(POTENTIOMETER_PIN);
      position_wanted = encoder_lower_threshold + pot_reading * (encoder_higher_threshold - encoder_lower_threshold) / 4096.0;
      Serial.print("\n\n");
      Serial.println(position_wanted);
      Serial.println(encoder.getPosition());
      setMotorSpeed(MOTOR_A, MOTOR_B, -speed);
    }
  }
  stopMotors(MOTOR_A, MOTOR_B);
  
  if (position_wanted > encoder.getPosition()) {
    while (position_wanted - 0.05 > encoder.getPosition()) {
      pot_reading = analogRead(POTENTIOMETER_PIN);
      position_wanted = encoder_lower_threshold + pot_reading * (encoder_higher_threshold - encoder_lower_threshold) / 4096.0;
      Serial.print("\n\n");
      Serial.println(position_wanted);
      Serial.println(encoder.getPosition());
      setMotorSpeed(MOTOR_A, MOTOR_B, speed);
    }
  }
  stopMotors(MOTOR_A, MOTOR_B);
}

void setRadialPosition(int percent_rotated, int MOTOR_A, int MOTOR_B, double encoder_higher_threshold, double encoder_lower_threshold, I2CEncoder encoder, int speed) {
  
  double position_wanted = encoder_lower_threshold + percent_rotated * (encoder_higher_threshold - encoder_lower_threshold) / 100;

  stopMotors(MOTOR_A, MOTOR_B);

  if (position_wanted < encoder.getPosition()) {
    while (position_wanted + 0.05 < encoder.getPosition()) {
      Serial.print("\n\n");
      Serial.println(position_wanted);
      Serial.println(encoder.getPosition());
      setMotorSpeed(MOTOR_A, MOTOR_B, -speed);
    }
  }
  stopMotors(MOTOR_A, MOTOR_B);
  
  if (position_wanted > encoder.getPosition()) {
    while (position_wanted + 0.1 > encoder.getPosition()) {
      Serial.print("\n\n");
      Serial.println(position_wanted);
      Serial.println(encoder.getPosition());  
      setMotorSpeed(MOTOR_A, MOTOR_B, speed);
    
    }
  }
  stopMotors(MOTOR_A, MOTOR_B);
}

void extendArm(int LINEAR_MOTOR_AA, int LINEAR_MOTOR_BB, int EXTENDED_LIMIT_SWITCH, int speed) {
  setMotorSpeed(LINEAR_MOTOR_AA, LINEAR_MOTOR_BB, abs(speed));
  while (!digitalRead(EXTENDED_LIMIT_SWITCH)) {}
  stopMotors(LINEAR_MOTOR_AA, LINEAR_MOTOR_BB);
}

void contractArm(int LINEAR_MOTOR_AA, int LINEAR_MOTOR_BB, int CONTRACTED_LIMIT_SWITCH, int speed) {
  setMotorSpeed(LINEAR_MOTOR_AA, LINEAR_MOTOR_BB, -abs(speed));
  while (!digitalRead(CONTRACTED_LIMIT_SWITCH)) {}
  stopMotors(LINEAR_MOTOR_AA, LINEAR_MOTOR_BB);
}

void startPosition(int RADIAL_MOTOR_AA, int RADIAL_MOTOR_BB, double radial_encoder_higher_threshold, double radial_encoder_lower_threshold, I2CEncoder radial_encoder, 
int LINEAR_MOTOR_AA, int LINEAR_MOTOR_BB, double linear_encoder_higher_threshold, double linear_encoder_lower_threshold) {
  contractArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, CONTRACTED_SWITCH, 200);
  setRadialPosition(0, RADIAL_MOTOR_AA, RADIAL_MOTOR_BB, radial_encoder_higher_threshold, radial_encoder_lower_threshold, radial_encoder, 100);
}

void rake(int RADIAL_MOTOR_AA, int RADIAL_MOTOR_BB, double radial_encoder_higher_threshold, double radial_encoder_lower_threshold, I2CEncoder radial_encoder, 
int LINEAR_MOTOR_AA, int LINEAR_MOTOR_BB, double linear_encoder_higher_threshold, double linear_encoder_lower_threshold) {
  extendArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, EXTENDED_SWITCH, 200);
  setRadialPosition(0, RADIAL_MOTOR_AA, RADIAL_MOTOR_BB, radial_encoder_higher_threshold, radial_encoder_lower_threshold, radial_encoder, 100);
  extendArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, EXTENDED_SWITCH, 200);
  setRadialPosition(100, RADIAL_MOTOR_AA, RADIAL_MOTOR_BB, radial_encoder_higher_threshold, radial_encoder_lower_threshold, radial_encoder, 100);
  contractArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, CONTRACTED_SWITCH, 200);
}

void pushOffPlate(int RADIAL_MOTOR_AA, int RADIAL_MOTOR_BB, double radial_encoder_higher_threshold, double radial_encoder_lower_threshold, I2CEncoder radial_encoder, 
int LINEAR_MOTOR_AA, int LINEAR_MOTOR_BB, double linear_encoder_higher_threshold, double linear_encoder_lower_threshold) {

}

void radialHoming() {
  encoder.zero();
  stopMotors(RADIAL_MOTOR_A, RADIAL_MOTOR_B);
}

void setup() {
  pinMode(LINEAR_MOTOR_A, OUTPUT);
  pinMode(RADIAL_MOTOR_A, OUTPUT);
  pinMode(LINEAR_MOTOR_B, OUTPUT);
  pinMode(RADIAL_MOTOR_B, OUTPUT);

  pinMode(4, INPUT);
  pinMode(5, INPUT);

  linear_reading = 0;
  radial_reading = 0;

  Serial.begin(9600);
  Wire.begin(17, 18);
  encoder.init(MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder.zero();

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();

  start_millis = millis();
  encoder_count = 0;

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.display();

  //attachInterrupt(digitalPinToInterrupt(RADIAL_HOMING_SWITCH), radialHoming, RISING);

  delay(1000);
}

void loop() {
  current_millis = millis();
  delay(5);

  extended_button_pressed = digitalRead(4);
  contracted_button_pressed = digitalRead(CONTRACTED_SWITCH);

  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.display();


display_handler.clearDisplay();
display_handler.printf("The button is \n");



    if (digitalRead(4) == HIGH) {
      display_handler.printf("CLOSED");
    } else {
      display_handler.printf("OPEN");
    }

    display_handler.display();

  //extendArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, EXTENDED_SWITCH, 200);
  //contractArm(LINEAR_MOTOR_A, LINEAR_MOTOR_B, CONTRACTED_SWITCH, 200);

/*
  if (extended_button_pressed) {
    stopMotors(LINEAR_MOTOR_A, LINEAR_MOTOR_B);
  } else {
    setMotorSpeed(LINEAR_MOTOR_A, LINEAR_MOTOR_B, 200);
  }
*/
  //setMotorSpeed(LINEAR_MOTOR_A, LINEAR_MOTOR_B, 100);
  
  //startPosition(RADIAL_MOTOR_A, RADIAL_MOTOR_B, 0, -1.1, encoder, 100, LINEAR_MOTOR_A, LINEAR_MOTOR_B, 45.0, 5.0, 100);
  //pickupAndRake(RADIAL_MOTOR_A, RADIAL_MOTOR_B, 0, -1.1, encoder, LINEAR_MOTOR_A, LINEAR_MOTOR_B, 35.0, 2.0);

  //setRadialPosition(100, BED_MOTOR_A, BED_MOTOR_B, 0.4, 0, encoder, 15);
  //setMotorSpeed(BED_MOTOR_A, 200, BED_MOTOR_B, 0);


/*
  if (current_millis > start_millis + 50) {
    start_millis = millis();
    display_handler.clearDisplay();
    /*display_handler.printf("Encoder position is \n");
    display_handler.printf("%.2f", encoder.getPosition());
    
    
    display_handler.printf("The button is \n");

    if (extended_button_pressed == HIGH) {
      display_handler.printf("CLOSED");
    } else {
      display_handler.printf("OPEN");
    }

    display_handler.printf("\n\nThe contracted button is \n");

    if (contracted_button_pressed) {
      display_handler.printf("CLOSED");
    } else {
      display_handler.printf("OPEN");
    }
    
    display_handler.display();
  }
  */
}
