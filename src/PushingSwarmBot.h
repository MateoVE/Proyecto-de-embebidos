#ifndef PUSHING_SWARMBOT_H
#define PUSHING_SWARMBOT_H

#include <ESP32Servo.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>

#define OCCLUDED_THRESHOLD 50

class PushingSwarmBot {
private:
    // Servo objects
    Servo leftServo, rightServo;

    // Encoder objects
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;

    // PID objects
    double distanceSetpoint, distanceInput, distanceOutput;
    double angleSetpoint, angleInput, angleOutput;
    PID distancePID;
    PID anglePID;

    // Robot physical parameters
    const double WHEEL_DIAMETER = 65.0;  // mm
    const double WHEEL_BASE = 150.0;     // mm
    const double COUNTS_PER_REV = 20.0;

    // Sensors
    const int NUM_IR_SENSORS = 8;
    const int NUM_LIGHT_SENSORS = 8;
    const int LIGHT_SENSOR_PINS[8] = {36, 37, 38, 39, 40, 41, 42, 43};
    const int IR_SENSOR_PINS[8] = {44, 45, 46, 47, 48, 49, 50, 51};
    bool is_occluded = false;
    int light_angle = 0;

    // Lévy flight parameters
    float alpha = 1.0;
    float mu = 1.5;
    float beta = 0.8;

    // Movement parameters
    const int MAX_PUSH_SPEED = 90;
    const int SEARCH_SPEED = 70;
    const int MAX_SEARCH_DISTANCE = 500;

    bool object_detected = false;
    unsigned long last_light_check = 0;
    const unsigned long LIGHT_CHECK_INTERVAL = 500;

    // Private helper methods
    double getDistance();
    double getAngle();
    bool isObstacleDetected();
    int getAlignmentCorrection();
    bool isObjectDetected();

public:
    PushingSwarmBot();
    
    void begin(int leftServoPin, int rightServoPin, 
               int leftEncoderPinA, int leftEncoderPinB,
               int rightEncoderPinA, int rightEncoderPinB);

    bool getIsOccluded();
    int getLightDirection();
    bool moveDistance(double targetDistance);
    bool turnToAngle(double targetAngle);
    void setMotorSpeeds(double leftSpeed, double rightSpeed);
    void resetEncoders();
    void tuneDistancePID(double kp, double ki, double kd);
    void tuneAnglePID(double kp, double ki, double kd);
    void findLightDirection();
    int levy_angle();
    int levy_step();
    bool face_light();
    bool find_object();
    bool push_object();
};

#endif // PUSHING_SWARMBOT_H