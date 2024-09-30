

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-input-data-html-form/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// I (Ron) have also modified this significantly.

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

#include "configServer.h"
#include "encodedDrivetrain.h"

#define MAX_NUMBER_OF_TWEAKABLE_VARIABLES 20

#define MAX_NUMBER_OF_VALUE_READOUTS 20

void (*callbacks[MAX_NUMBER_OF_TWEAKABLE_VARIABLES]) (double newValue); // array of callback function pointers
const char * varNames[MAX_NUMBER_OF_TWEAKABLE_VARIABLES];
double values[MAX_NUMBER_OF_TWEAKABLE_VARIABLES];
int nextVariableSlot = 0;

double (*readoutCallbacks[MAX_NUMBER_OF_VALUE_READOUTS]) ();
const char * readoutNames[MAX_NUMBER_OF_VALUE_READOUTS];
int nextReadoutSlot = 0;


AsyncWebServer server(80);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Honor 6X";
const char* password = "1234512345";

const char* PARAM_INPUT_O = "O";
const char* PARAM_INPUT_P = "P";
const char* PARAM_INPUT_I = "I";
const char* PARAM_INPUT_D = "D";

void addTweakableVariable(const char * name, void (*callback)(double newValue), double initialValue) {
  varNames[nextVariableSlot] = name;
  callbacks[nextVariableSlot] = callback;
  values[nextVariableSlot] = initialValue;
  nextVariableSlot++;
}

void addValueReadout(const char * name, double (*callback)()) {
  readoutNames[nextReadoutSlot] = name;
  readoutCallbacks[nextReadoutSlot] = callback;
  nextReadoutSlot++;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncResponseStream * generateResponse(AsyncWebServerRequest * request) {
  AsyncResponseStream * response = request->beginResponseStream("text/html");
  response->print(R"rawliteral(
  <!DOCTYPE HTML><html><head>
    <title>ESP Input Form</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    </head><body>
    )rawliteral");
  
  for (int i = 0; i < nextVariableSlot; i++) {
    response->printf(R"rawliteral(
    <form action="/">
      %s: %f: <input type="text" name="%d">
      <input type="submit" value="Submit">
    </form><br>
    )rawliteral", varNames[i], values[i], i);
  }

  for (int i = 0; i < nextReadoutSlot; i++) {
    double value = readoutCallbacks[i]();
    response->printf("%s: %f <br>", readoutNames[i], value);
  }

  response->print("</body></html>");

  return response;
}

void initConfigServer() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_15dBm); // Apparently reduced TX power helps
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

    if (request->params() > 0) {

      double received_value = request->getParam(0)->value().toDouble();

      int param_num = request->getParam(0)->name().toInt();

      values[param_num] = received_value;

      (callbacks[param_num])(received_value); // run the appropriate callback

      Serial.println(received_value);
    }

    request->send(generateResponse(request));
  });

  server.onNotFound(notFound);
  server.begin();
}
