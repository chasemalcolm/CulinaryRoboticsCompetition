#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define PULSE_PIN 8
#define PULSE_FREQ_HZ 1000

#define BUFFER_SIZE 100
#define FREQUENCY_HZ 1000
#define SIGNAL_IN_PIN_1 6
#define SIGNAL_IN_PIN_2 7
#define TIME_STEP_US 200 // 100 is too slow if you have two beacons on ESP32-S3
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int samples1[BUFFER_SIZE];
int samples2[BUFFER_SIZE];
long firstPollTime;

int nextSampleWriteIndex = 0;// index into a buffer of samples

// returns true if buffer is full
bool read_new_sample() {
  int sample1 = analogRead(SIGNAL_IN_PIN_1);
  int sample2 = analogRead(SIGNAL_IN_PIN_2);
  samples1[nextSampleWriteIndex] = sample1;
  samples2[nextSampleWriteIndex] = sample2;
  nextSampleWriteIndex++;
  return nextSampleWriteIndex == BUFFER_SIZE;
}

double get_cosine_sample(int sampleNum, double frequency_Hz) {
  return cos(2.0 * PI * frequency_Hz * sampleNum * TIME_STEP_US / 1000000);
}

double get_sine_sample(int sampleNum, double frequency_Hz) {
  return sin(2.0 * PI * frequency_Hz * sampleNum * TIME_STEP_US / 1000000);
}

double get_correlation_strength(double frequency_Hz, int samples[]) {
  double sineSum = 0;
  double cosineSum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sineSum += samples[i] * get_sine_sample(i, frequency_Hz);
    cosineSum += samples[i] * get_cosine_sample(i, frequency_Hz);
  }
  return sqrt(pow(sineSum, 2) + pow(cosineSum, 2));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(17, 18);

  pinMode(PULSE_PIN, OUTPUT);
  tone(PULSE_PIN, PULSE_FREQ_HZ);

  pinMode(SIGNAL_IN_PIN_1, INPUT);
  pinMode(SIGNAL_IN_PIN_2, INPUT);

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.clearDisplay();

  // the buffer should be BUFFER_NUM_PERIODS long
  // ideally this should be an integer number of us
  //int timeStep_us = TIME_STEP_US; //1000000 / FREQUENCY_HZ * BUFFER_NUM_PERIODS / BUFFER_SIZE;
}

void loop() {
  nextSampleWriteIndex = 0;
    bool bufferFull = false;
    long nextPollTime = micros();
    while (! bufferFull) {
      if (micros() >= nextPollTime) {
        bufferFull = read_new_sample();
        nextPollTime += TIME_STEP_US;
      }
    }
    
    double frequencyStrength1 = get_correlation_strength(FREQUENCY_HZ, samples1);
    double frequencyStrength2 = get_correlation_strength(FREQUENCY_HZ, samples2);

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.printf("IR1 strength: \n");
    display_handler.println(frequencyStrength1);
    display_handler.printf("IR2 strength: \n");
    display_handler.println(frequencyStrength2);
    display_handler.display();

}
