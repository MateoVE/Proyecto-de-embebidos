

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

/*
#include "TestSensors.h"

Tests robot;

void setup() {
    robot.setup();
    Serial.println("Set up done");
}

void loop() {
    
    Serial.println("oled?");
    robot.testOled();
    
    delay(2000);
    Serial.println("lights?");
    robot.testLight(); 

    delay(2000);
    Serial.println("IRsensors?");
    robot.testDist(); 
    delay(2000);

    //Serial.println("Motors?");
    //robot.testServos();

    //Serial.println("ENCODERS?");
    //robot.monitorEncoders(); 
    

    // loopAP();
    // robot.MsgOverWifi();
    // delay(1000);

    //robot.testServos();
    //delay(2000);
    //robot.stopServos();
    //delay(500);

}


*/
