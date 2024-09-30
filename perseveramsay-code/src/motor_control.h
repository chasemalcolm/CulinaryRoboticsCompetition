// call this before using the motors, usually in setup()!
void init_motor_control();

// valid LEDC channels are 0-7. Use only one for each motor.
// Power is in range -1 - 1, scaled according to maxPower, which is in range 0-1
void run_motor(int pinForward, int pinBackward, double power, double maxPower, int LEDCChannel);

void steer_drivetrain(double forward, double turning);

void run_drive_motors(double leftPower, double rightPower);

void xSlide(double power);

void ySlide(double power);

void pryFry(double position, double power);

void tickFryPry();

void moveSpatula(double position);