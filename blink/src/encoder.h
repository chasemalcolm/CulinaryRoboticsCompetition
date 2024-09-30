#pragma once

/*
 * When creating an Encoder, pass a function which calls update() on the encoder, to be used for an ISR.
 * In this case, pin A rising first indicated forward motion
 * To use, initialize the object and then call begin() no earlier than void setup()
*/
class Encoder {  
  public:
  volatile int position;

  private:
  int pinA;
  int pinB;
  volatile bool stateA;
  volatile bool stateB;
  void (* updater)();

  public:
  Encoder(int pinA_, int pinB_, void (*updater_)());

  void begin();

  void update();
};