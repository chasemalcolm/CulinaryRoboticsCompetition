#pragma once

// This intis both wifi and the server.
void initConfigServer();

// Adds a new variable to the list of tweakable variables. Variables are always of type double. the provided callback function needs to 
void addTweakableVariable(const char * name, void (*callback)(double newValue), double initialValue);

// Adds a new value readout. the callback function should supply the value to print on demand.
void addValueReadout(const char * name, double (*callback)());