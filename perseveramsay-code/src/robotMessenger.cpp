#include "robotMessenger.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "rgbled.h"

// This shound be the target's MAC address
// uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xE1, 0xF7, 0xA8}; // "A" Aliexpress unit
// uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0xCA, 0x60, 0x38}; // Unlabelled AliExpress unit
uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x5B, 0x06, 0x58}; // Legit ESP with broken Serial port
// uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x5B, 0x44, 0x80}; // Legit ESP with working Serial port

struct RobotMessage {
    Phase phase;
};

esp_now_peer_info_t peerInfo;

Phase currentPhase = START;

// Stores the time the phase was reached for each phase
long phaseReachedMillis[NUM_PHASES];

bool usingBothRobots;

Phase getCurrentPhase() {
    if (!usingBothRobots) {
      return NUM_PHASES;
    }
    
    return currentPhase;
}

long getPhaseReachedMillis(Phase phase) {
    return phaseReachedMillis[phase];
}

void enterPhase(Phase phase) {
    currentPhase = phase;
    phaseReachedMillis[phase] = millis();
}

void sendRobotMessage(Phase phase) {
    if (!usingBothRobots) {
      return;
    }

    enterPhase(phase);
    RobotMessage message = {phase};
    // the data being sent does not need to be valid after esp_now_send returns, so it's fine to give it a pointer to a local variable.
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
    if (result == ESP_OK) {
        //Serial.println("Sent with success. ");
    } else {
        //Serial.println("Error sending data");
        setLedColor(LED_RED);
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail\n");
  if (status != ESP_NOW_SEND_SUCCESS) {
    setLedColor(LED_ORANGE);
  }
}

void OnDataRecv(const uint8_t * mac, const RobotMessage *incomingData, int len) {
    enterPhase(incomingData->phase);
    // Serial.printf("Received %d", incomingData->phase);
}

void initRobotMessenger(bool usingBothRobots_) {
  usingBothRobots = usingBothRobots_;
  
  if (!usingBothRobots) {
    return;
  } 

  // Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    setLedColor(LED_RED);
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  #ifdef WIFI_DISABLED
    peerInfo.channel = WIFI_CHANNEL;
  #else
    peerInfo.channel = 0; // use current channel
  #endif

  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    // Serial.println("Failed to add peer");
    setLedColor(LED_RED);
    return;
  }
}