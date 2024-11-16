#include "PushingSwarmBot.h"

PushingSwarmBot robot;

void setup() {
    robot.setup();
    Serial.println("Set up done");
}

void loop() {
    robot.controlLoop(); 
    
}