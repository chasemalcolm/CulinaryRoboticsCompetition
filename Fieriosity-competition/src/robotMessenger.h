// comment this out when using hotspots/OTA/ConfigServer
#define WIFI_DISABLED
// if WIFI_DISABLED, this indicates the wifi channel to use for ESP-NOW
// usually you only use channels 1, 6, or 11
#define WIFI_CHANNEL 0


enum Phase {
  START = 0,
  BOTTOM_BUN_READY,
  PATTY_READY,
  FRY_READY,
  TOP_BUN_READY,
  FRY_2_READY,
  NUM_PHASES // This is a dummy phase used to count the number of phases for array sizing purposes
};

void initRobotMessenger();

Phase getCurrentPhase();

void sendRobotMessage(Phase phase);

long getPhaseReadchedMillis(Phase phase);