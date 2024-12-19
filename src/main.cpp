/*

#include "PushingSwarmBot.h"

PushingSwarmBot robot;

void setup() {
    robot.setup();
    Serial.println("Set up done");
}

void loop() {
    robot.controlLoop(); 
    
}

*/

#include "TestSensors.h"

Tests robot;

void setup() {
    robot.setup();
    Serial.println("Set up done");
}

void loop() {
    Serial.println("oled?");
    robot.testOled();

    Serial.println("lights?");
    robot.testLight(); 

    Serial.println("IRsensors?");
    robot.testDist(); 

    Serial.println("Motors?");
    robot.testServos();

    Serial.println("ENCODERS?");
    robot.monitorEncoders(); 
    loopAP();
    robot.MsgOverWifi();
    delay(1000);

    robot.stopServos();
    delay(500);

}

