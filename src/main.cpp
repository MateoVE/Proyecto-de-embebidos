#include "PushingSwarmBot.h"

PushingSwarmBot robot;

void setup() {
    robot.setup();
    Serial.println("Set up done");
}

void loop() {
    robot.controlLoop(); 
    
    /*
    Serial.println("Testing");
    delay(500);
    robot.test_turn(90);
    robot.test_move(10);
    Serial.println("wait");
    delay(3000);
    */

    
}
