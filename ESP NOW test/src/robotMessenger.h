typedef enum Phase {
  START = 0,
  BOTTOM_BUN_READY,
  PATTY_READY,
  TOP_BUN_READY,
  NUM_PHASES // This is a dummy phase used to count the number of phases for array sizing purposes
};

void initRobotMessenger();

Phase getCurrentPhase();

void sendRobotMessage(Phase phase);