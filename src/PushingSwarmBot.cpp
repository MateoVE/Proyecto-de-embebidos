#include "PushingSwarmBot.h"

// Constructor
PushingSwarmBot::PushingSwarmBot() : 
    distancePID(&distanceInput, &distanceOutput, &distanceSetpoint, 2, 0.1, 0.5, DIRECT),
    anglePID(&angleInput, &angleOutput, &angleSetpoint, 2.5, 0.1, 0.8, DIRECT)
{
    distanceSetpoint = 0;
    angleSetpoint = 0;
}

void PushingSwarmBot::begin(int leftServoPin, int rightServoPin, 
                           int leftEncoderPinA, int leftEncoderPinB,
                           int rightEncoderPinA, int rightEncoderPinB) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    leftServo.setPeriodHertz(50);
    rightServo.setPeriodHertz(50);
    leftServo.attach(leftServoPin);
    rightServo.attach(rightServoPin);

    leftEncoder.attachFullQuad(leftEncoderPinA, leftEncoderPinB);
    rightEncoder.attachFullQuad(rightEncoderPinA, rightEncoderPinB);

    distancePID.SetMode(AUTOMATIC);
    anglePID.SetMode(AUTOMATIC);
    distancePID.SetOutputLimits(-90, 90);
    anglePID.SetOutputLimits(-90, 90);
    distancePID.SetSampleTime(20);
    anglePID.SetSampleTime(20);
}

double PushingSwarmBot::getDistance() {
    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();
    return ((leftCount + rightCount) / 2.0) * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
}

double PushingSwarmBot::getAngle() {
    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();
    double leftDist = leftCount * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
    double rightDist = rightCount * (PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
    return ((rightDist - leftDist) / WHEEL_BASE) * (180.0 / PI);
}

bool PushingSwarmBot::isObstacleDetected() {
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        if (digitalRead(IR_SENSOR_PINS[i]) == LOW) {
            return true;
        }
    }
    return false;
}

bool PushingSwarmBot::getIsOccluded() {
    return is_occluded;
}

int PushingSwarmBot::getLightDirection() {
    return light_angle;
}

bool PushingSwarmBot::moveDistance(double targetDistance) {
    static bool isMoving = false;

    if (!isMoving) {
        resetEncoders();
        isMoving = true;
    }

    distanceSetpoint = targetDistance;
    distanceInput = getDistance();

    bool targetReached = abs(targetDistance - distanceInput) < 2.0;
    if (targetReached) {
        setMotorSpeeds(0, 0);
        isMoving = false;
        return targetReached;
    }

    if (isObstacleDetected()) {
        setMotorSpeeds(0, 0);
        isMoving = false;
        return true;
    }

    if (distancePID.Compute()) {
        setMotorSpeeds(distanceOutput, distanceOutput);
    }

    return false;
}

bool PushingSwarmBot::turnToAngle(double targetAngle) {
    static bool isTurning = false;
    
    if (!isTurning) {
        resetEncoders();
        isTurning = true;
    }
    
    double currentRotation = getAngle();
    angleSetpoint = targetAngle;
    angleInput = currentRotation;
    
    if (anglePID.Compute()) {
        setMotorSpeeds(-angleOutput, angleOutput);
    }
    
    bool finished = abs(targetAngle - currentRotation) < 2.0;
    if (finished) {
        isTurning = false;
        setMotorSpeeds(0, 0);
    }
    return finished;
}

void PushingSwarmBot::setMotorSpeeds(double leftSpeed, double rightSpeed) {
    leftServo.write(map(leftSpeed, -90, 90, 0, 180));
    rightServo.write(map(rightSpeed, -90, 90, 180, 0));
}

void PushingSwarmBot::resetEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

void PushingSwarmBot::tuneDistancePID(double kp, double ki, double kd) {
    distancePID.SetTunings(kp, ki, kd);
}

void PushingSwarmBot::tuneAnglePID(double kp, double ki, double kd) {
    anglePID.SetTunings(kp, ki, kd);
}

void PushingSwarmBot::findLightDirection() {
    int maxLight = 0;
    int maxSensor = 0;

    for(int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        int reading = analogRead(LIGHT_SENSOR_PINS[i]);
        if(reading > maxLight) {
            maxLight = reading;
            maxSensor = i;
        }
    }

    is_occluded = (maxLight <= OCCLUDED_THRESHOLD);
    light_angle = maxSensor * 45;

    while (light_angle > 180) light_angle -= 360;
    while (light_angle < -180) light_angle += 360;
}

int PushingSwarmBot::levy_angle() {
    int random_angle = (rand() % 361) - 180;
    
    float diff = abs(random_angle - light_angle);
    if (diff > 180) {
        diff = 360 - diff;
    }
    
    if (diff < 60) {
        int opposite_light = (light_angle > 0) ? light_angle - 180 : light_angle + 180;
        int spread = 90;
        random_angle = opposite_light + (rand() % (2 * spread + 1)) - spread;
        
        if (random_angle > 180) random_angle -= 360;
        if (random_angle < -180) random_angle += 360;
    }
    
    return random_angle;
}

int PushingSwarmBot::levy_step() {
    float u = rand() / (float)RAND_MAX;
    if (u == 0.0) u = 0.0001;
    float step = alpha * pow(u, -1.0 / mu);
    return (int)step;
}

bool PushingSwarmBot::face_light() {
    findLightDirection();
    resetEncoders();

    while (!turnToAngle(light_angle)) {
        if (!is_occluded) {
            return false;
        }
        delay(10);
    }
    return true;
}

bool PushingSwarmBot::find_object() {
    resetEncoders();
    double current_distance = 0;

    while (current_distance < MAX_SEARCH_DISTANCE) {
        if (millis() - last_light_check > LIGHT_CHECK_INTERVAL) {
            findLightDirection();
            last_light_check = millis();

            if (!getIsOccluded()) {
                setMotorSpeeds(0, 0);
                return false;
            }
        }

        if (isObjectDetected()) {
            setMotorSpeeds(0, 0);
            return true;
        }

        setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);
        current_distance = getDistance();

        if (current_distance >= MAX_SEARCH_DISTANCE) {
            setMotorSpeeds(0, 0);
            return false;
        }

        delay(10);
    }

    setMotorSpeeds(0, 0);
    return false;
}

bool PushingSwarmBot::push_object() {
    if (!isObjectDetected()) {
        return false;
    }

    if (millis() - last_light_check > LIGHT_CHECK_INTERVAL) {
        findLightDirection();
        last_light_check = millis();

        if (!getIsOccluded()) {
            setMotorSpeeds(0, 0);
            return false;
        }
    }

    int correction = getAlignmentCorrection();
    int leftSpeed = MAX_PUSH_SPEED - (correction * 10);
    int rightSpeed = MAX_PUSH_SPEED + (correction * 10);

    leftSpeed = constrain(leftSpeed, 0, MAX_PUSH_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_PUSH_SPEED);

    setMotorSpeeds(leftSpeed, rightSpeed);
    return true;
}

bool PushingSwarmBot::isObjectDetected() {
    return (digitalRead(IR_SENSOR_PINS[3]) == LOW || 
            digitalRead(IR_SENSOR_PINS[4]) == LOW);
}

int PushingSwarmBot::getAlignmentCorrection() {
    bool leftSensor = digitalRead(IR_SENSOR_PINS[3]) == LOW;
    bool rightSensor = digitalRead(IR_SENSOR_PINS[4]) == LOW;

    if (leftSensor && !rightSensor) return -1;
    if (!leftSensor && rightSensor) return 1;
    return 0;
}