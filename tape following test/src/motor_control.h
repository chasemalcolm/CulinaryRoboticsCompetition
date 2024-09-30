// call this before using the motors, usually in setup()!
void init_motor_control();

// valid LEDC channels are 0-7. Use only one for each motor.
// Power is in range -1 - 1, scaled according to maxPower, which is in range 0-1
void run_motor(int pinForward, int pinBackward, double power, double maxPower, int LEDCChannel);

void steer_drivetrain(double forward, double turning);


// These all take power in range 0-1
void run_bed(double power);

void run_arm_rotation(double power);

void run_arm_extension(double power);



