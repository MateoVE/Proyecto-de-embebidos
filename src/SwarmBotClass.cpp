#include "SwarmBotClass.h"

PushingSwarmBot::PushingSwarmBot() :
    distancePID(&distanceInput, &distanceOutput, &distanceSetpoint, 2, 0.1, 0.5, DIRECT),
    anglePID(&angleInput, &angleOutput, &angleSetpoint, 2.5, 0.1, 0.8, DIRECT),
    alpha(1.0), mu(1.5), beta(0.8), is_occluded(false), light_angle(0)
{
    distanceSetpoint = 0;
    angleSetpoint = 0;
}

void PushingSwarmBot::begin(int leftServoPin, int rightServoPin, int leftEncoderPinA, int leftEncoderPinB, int rightEncoderPinA, int rightEncoderPinB) {
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
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (digitalRead(IR_SENSOR_PINS[i]) == LOW) {
            return true;
        }
    }
    return false;
}

void PushingSwarmBot::setMotorSpeeds(double leftSpeed, double rightSpeed) {
    leftServo.write(map(leftSpeed, -90, 90, 0, 180));
    rightServo.write(map(rightSpeed, -90, 90, 180, 0));
}

void PushingSwarmBot::resetEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

bool PushingSwarmBot::moveDistance(double targetDistance) {
    distanceSetpoint = targetDistance;
    distanceInput = getDistance();

    if (abs(targetDistance - distanceInput) < 2.0) {
        setMotorSpeeds(0, 0);
        return true;
    }

    if (isObstacleDetected()) {
        setMotorSpeeds(0, 0);
        return true;
    }

    if (distancePID.Compute()) {
        setMotorSpeeds(distanceOutput, distanceOutput);
    }

    return false;
}

bool PushingSwarmBot::turnToAngle(double targetAngle) {
    angleSetpoint = targetAngle;
    angleInput = getAngle();

    if (anglePID.Compute()) {
        setMotorSpeeds(-angleOutput, angleOutput);
    }

    return abs(targetAngle - angleInput) < 2.0;
}

void PushingSwarmBot::findLightDirection() {
    int maxLight = 0;
    int maxSensor = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int reading = analogRead(SENSOR_PINS[i]);
        if (reading > maxLight) {
            maxLight = reading;
            maxSensor = i;
        }
    }

    is_occluded = (maxLight <= OCCLUDED_THRESHOLD);
    light_angle = (maxSensor * 45) - 180;
}

int PushingSwarmBot::levy_angle() {
    int random_angle = rand() % 360;
    float diff = fabs(random_angle - light_angle);
    float angle_diff = (diff > 180) ? (360 - diff) : diff;
    float bias_term = 1.0 - beta * (angle_diff / 180.0);

    return ((rand() / (float)RAND_MAX) < bias_term) ? (random_angle + 180) % 360 : random_angle;
}

int PushingSwarmBot::levy_step() {
    float u = rand() / (float)RAND_MAX;
    if (u == 0.0) u = 0.0001;

    return (int)(alpha * pow(u, -1.0 / mu));
}

bool PushingSwarmBot::push_object() {
    if (!isObjectDetected()) {
        return false;
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
    return (digitalRead(IR_SENSOR_PINS[3]) == LOW || digitalRead(IR_SENSOR_PINS[4]) == LOW);
}

int PushingSwarmBot::getAlignmentCorrection() {
    bool leftSensor = digitalRead(IR_SENSOR_PINS[3]) == LOW;
    bool rightSensor = digitalRead(IR_SENSOR_PINS[4]) == LOW;

    return leftSensor && !rightSensor ? -1 : (!leftSensor && rightSensor ? 1 : 0);
}