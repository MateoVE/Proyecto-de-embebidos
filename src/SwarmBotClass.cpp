#include <ESP32Servo.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>

#define OCCLUDED_THRESHOLD 50

class LibraryDrivebot {
private:
    // Servo objects
    Servo leftServo, rightServo;

    // Encoder objects (using ESP32Encoder library)
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;

    // PID objects (using PID_v1 library)
    double distanceSetpoint, distanceInput, distanceOutput;
    double angleSetpoint, angleInput, angleOutput;
    PID distancePID;
    PID anglePID;

    // Robot physical parameters
    const double WHEEL_DIAMETER = 65.0;  // mm
    const double WHEEL_BASE = 150.0;     // mm
    const double COUNTS_PER_REV = 20.0;

    // Convert encoder counts to distance in mm
    double getDistance() {
        long leftCount = leftEncoder.getCount();
        long rightCount = rightEncoder.getCount();
        return ((leftCount + rightCount) / 2.0) * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
    }

    // Convert encoder counts to angle in degrees
    double getAngle() {
        long leftCount = leftEncoder.getCount();
        long rightCount = rightEncoder.getCount();
        double leftDist = leftCount * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
        double rightDist = rightCount * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
        return ((rightDist - leftDist) / WHEEL_BASE) * (180.0 / PI);
    }

    // Light sensors
    const int NUM_SENSORS = 8;
    const int SENSOR_PINS[8] = {36, 37, 38, 39, 40, 41, 42, 43}; // Adjust pins
    const int IR_SENSOR_PINS[8] = {44, 45, 46, 47, 48, 49, 50, 51};  // Add your actual pins
    bool is_occluded = false;
    int light_angle = 0;

    // Define parameters for Lévy flight
    float alpha = 1.0;          // Scaling factor
    float mu = 1.5;             // Power-law exponent
    float beta = 0.8;           // Directional bias parameter

    // Check if any IR sensor detects an obstacle
    bool isObstacleDetected() {
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (digitalRead(IR_SENSOR_PINS[i]) == LOW) {  // Assuming LOW means obstacle detected
                return true;
            }
        }
        return false;
    }

    const int MAX_PUSH_SPEED = 90;      // Maximum speed for pushing
    const int SEARCH_SPEED = 70;        // Speed for moving towards object
    const int MAX_SEARCH_DISTANCE = 500; // Maximum mm to search for object

    bool object_detected = false;
    unsigned long last_light_check = 0;
    const unsigned long LIGHT_CHECK_INTERVAL = 500; // Check light every 500ms


public:
    LibraryDrivebot() : 
        distancePID(&distanceInput, &distanceOutput, &distanceSetpoint, 2, 0.1, 0.5, DIRECT),
        anglePID(&angleInput, &angleOutput, &angleSetpoint, 2.5, 0.1, 0.8, DIRECT)
    {
        // Initialize PID setpoints
        distanceSetpoint = 0;
        angleSetpoint = 0;
    }

    void begin(int leftServoPin, int rightServoPin, 
               int leftEncoderPinA, int leftEncoderPinB,
               int rightEncoderPinA, int rightEncoderPinB) {
        // Setup servos
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        leftServo.setPeriodHertz(50);
        rightServo.setPeriodHertz(50);
        leftServo.attach(leftServoPin);
        rightServo.attach(rightServoPin);

        // Setup encoders
        leftEncoder.attachFullQuad(leftEncoderPinA, leftEncoderPinB);
        rightEncoder.attachFullQuad(rightEncoderPinA, rightEncoderPinB);

        // Configure PIDs
        distancePID.SetMode(AUTOMATIC);
        anglePID.SetMode(AUTOMATIC);
        distancePID.SetOutputLimits(-90, 90);
        anglePID.SetOutputLimits(-90, 90);
        distancePID.SetSampleTime(20);
        anglePID.SetSampleTime(20);
    }

    bool getIsOccluded() {
        return is_occluded;
    }

    int getLightDirection(){
        return light_angle;
    }

    // move + obstacle avoidance by stopping
    bool moveDistance(double targetDistance) {
        // Set target for this movement
        distanceSetpoint = targetDistance;

        // Get current position
        distanceInput = getDistance();

        // Check if we've reached the target (within 2mm tolerance)
        bool targetReached = abs(targetDistance - distanceInput) < 2.0;
        if (targetReached) {
            setMotorSpeeds(0, 0);  // Stop motors
            return true;
        }

        // Check for obstacles BEFORE computing new speeds
        if (isObstacleDetected()) {
            setMotorSpeeds(0, 0);  // Emergency stop
            return true; // Stop pushing if obstacle is detected
        }

        // Only update motor speeds if we haven't reached target and no obstacles
        if (distancePID.Compute()) {
            setMotorSpeeds(distanceOutput, distanceOutput);
        }

        // Return false to indicate we're still moving
        return false;
    }

    bool turnToAngle(double targetAngle) {
        angleSetpoint = targetAngle;
        angleInput = getAngle();

        if (anglePID.Compute()) {
            // Apply opposite speeds for turning
            setMotorSpeeds(-angleOutput, angleOutput);
        }

        return abs(targetAngle - angleInput) < 2.0;  // 2 degree tolerance
    }

    void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        // Convert from -90:90 to servo angles (0:180)
        leftServo.write(map(leftSpeed, -90, 90, 0, 180));
        rightServo.write(map(rightSpeed, -90, 90, 180, 0));  // Reversed
    }

    void resetEncoders() {
        leftEncoder.clearCount();
        rightEncoder.clearCount();
    }

    // Tune PID parameters
    void tuneDistancePID(double kp, double ki, double kd) {
        distancePID.SetTunings(kp, ki, kd);
    }

    void tuneAnglePID(double kp, double ki, double kd) {
        anglePID.SetTunings(kp, ki, kd);
    }

    // Calculate the direction of strongest light (-180 to +180 degrees)
    void findLightDirection() {
        int maxLight = 0;
        int maxSensor = 0;

        // Find brightest sensor
        for(int i = 0; i < NUM_SENSORS; i++) {
            int reading = analogRead(SENSOR_PINS[i]);
            if(reading > maxLight) {
                maxLight = reading;
                maxSensor = i;
            }
        }

        if (maxLight > OCCLUDED_THRESHOLD) is_occluded = false; else is_occluded = true;
        // Convert sensor number to angle
        // Sensor 0 is front, positive angles are clockwise
        light_angle = (maxSensor * 45) - 180;
    }

    // Simplified biased Lévy flight function
    int levy_angle() {
        // Generate a random direction angle for the Lévy walk (0-360 degrees)
        int random_angle = rand() % 360;

        // Calculate the angle difference between the random direction and the light source direction
        float diff = fabs(random_angle - light_angle);
        float angle_diff = (diff > 180) ? (360 - diff) : diff;

        // Calculate the bias term based on the angle difference
        // The closer the random angle is to the light source angle, the higher the bias to avoid it
        float bias_term = 1.0 - beta * (angle_diff / 180.0);

        // Determine the new movement angle
        if ((rand() / (float)RAND_MAX) < bias_term) {
            // Choose a direction away from the light (opposite of random_angle)
            return (random_angle + 180) % 360;
        } else {
            // Keep the random direction
            return random_angle;
        }
    }

    // Helper function to generate a random step length using power-law distribution
    int levy_step() {
        float u = rand() / (float)RAND_MAX;
        if (u == 0.0) u = 0.0001;  // Avoid division by zero

        // Generate the Lévy step as a float
        float step = alpha * pow(u, -1.0 / mu);

        // Truncate to an integer (round down) by casting
        return (int)step;
    }

    // Face the last known direction of light
    bool face_light() {
        findLightDirection(); // Update light direction
        resetEncoders();

        // Turn to face light direction
        while (!turnToAngle(getLightDirection())) {
            if (!getIsOccluded()) {
                return false; // Lost shadow while turning
            }
            delay(10);
        }
        return true;
    }

    // Move forward until object is detected or max distance reached
    bool find_object() {
        resetEncoders();
        double current_distance = 0;

        while (current_distance < MAX_SEARCH_DISTANCE) {
            // Check if we're still in shadow
            if (millis() - last_light_check > LIGHT_CHECK_INTERVAL) {
                findLightDirection();
                last_light_check = millis();

                if (!getIsOccluded()) {
                    setMotorSpeeds(0, 0);
                    return false; // Lost shadow while searching
                }
            }

            // Check front IR sensors for object
            if (isObjectDetected()) {
                setMotorSpeeds(0, 0);
                return true; // Found object
            }

            // Move forward at search speed
            setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);

            // Update distance moved
            current_distance = getDistance();

            // If we've gone too far without finding object
            if (current_distance >= MAX_SEARCH_DISTANCE) {
                setMotorSpeeds(0, 0);
                return false;
            }

            delay(10);
        }

        setMotorSpeeds(0, 0);
        return false;
    }

    // Push object while maintaining alignment
    bool push_object() {
        // If no object detected, can't push
        if (!isObjectDetected()) {
            return false;
        }

        // Check light periodically while pushing
        if (millis() - last_light_check > LIGHT_CHECK_INTERVAL) {
            findLightDirection();
            last_light_check = millis();

            if (!getIsOccluded()) {
                setMotorSpeeds(0, 0);
                return false; // Lost shadow while pushing
            }
        }

        // Get alignment correction (-1: left, 0: centered, 1: right)
        int correction = getAlignmentCorrection();

        // Calculate motor speeds with correction
        int leftSpeed = MAX_PUSH_SPEED - (correction * 10);
        int rightSpeed = MAX_PUSH_SPEED + (correction * 10);

        // Constrain speeds
        leftSpeed = constrain(leftSpeed, 0, MAX_PUSH_SPEED);
        rightSpeed = constrain(rightSpeed, 0, MAX_PUSH_SPEED);

        // Apply speeds
        setMotorSpeeds(leftSpeed, rightSpeed);
        return true;
    }

    // Helper function for detecting object with front sensors
    bool isObjectDetected() {
        // Check front sensors (assuming sensors 3,4 are front-center)
        return (digitalRead(IR_SENSOR_PINS[3]) == LOW || 
                digitalRead(IR_SENSOR_PINS[4]) == LOW);
    }

    // Get alignment correction for pushing
    int getAlignmentCorrection() {
        bool leftSensor = digitalRead(IR_SENSOR_PINS[3]) == LOW;
        bool rightSensor = digitalRead(IR_SENSOR_PINS[4]) == LOW;

        if (leftSensor && !rightSensor) return -1;  // Turn left
        if (!leftSensor && rightSensor) return 1;   // Turn right
        return 0;  // Centered or no object
    }
};