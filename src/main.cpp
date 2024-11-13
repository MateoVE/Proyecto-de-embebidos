// tengo que medir estas señales y controlar estas salidas

// INPUT
// 8 distance sensors analog
// 4 light sensors analog

// 2 encoders?
// accelerometer 3 pins

// OUTPUT
// 2 servos PWM
// 1 display

// Pin definitions
#define LEFT_SERVO_PIN 13
#define RIGHT_SERVO_PIN 12
#define LEFT_ENCODER_A 27
#define LEFT_ENCODER_B 26
#define RIGHT_ENCODER_A 25
#define RIGHT_ENCODER_B 33

#include "PushingSwarmBot.h"

PushingSwarmBot robot;

void setup() {
    robot.begin(LEFT_SERVO_PIN, RIGHT_SERVO_PIN,
                LEFT_ENCODER_A, LEFT_ENCODER_B,
                RIGHT_ENCODER_A, RIGHT_ENCODER_B);
}

void loop() {
    // Track the light source initially
    robot.findLightDirection();

    if (robot.getIsOccluded()) {
        // Three-step process: face light, find object, push object

        // Step 1: Face the light direction
        if (robot.face_light()) {
            // Step 2: Move forward until object is found
            if (robot.find_object()) {
                // Step 3: Push the object while maintaining alignment
                while (robot.push_object()) {
                    delay(10);
                    // push_object returns false if shadow is lost or object is lost
                }
            }
        }

    } else {
        // Generate a random step length and angle using Lévy flight
        int step_length = robot.levy_step();
        int turn_angle = robot.levy_angle();

        // Turn degrees
        while (!robot.turnToAngle(turn_angle)) {
            delay(10);
        }

        // Move forward mm
        while (!robot.moveDistance(step_length)) {
            delay(10);
        }
    }
}