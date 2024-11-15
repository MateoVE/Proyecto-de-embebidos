#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>
#include <ESP32Servo.h>

class PushingSwarmBot {
public:
    enum State {
        SENSING,            // read light sensors
        PLANNING,           // levy
        TURNING,            // turning levy angle
        MOVING,             // advancing levy steps
        TURNING_TO_OBJ,     // turning to face light
        MOVING_TO_OBJ,      // moving to object, until hitting wall
        PUSHING            // pushing object
    };

    PushingSwarmBot() {
        is_occluded = false;
        object_detected = false;
        light_angle = 0;
        step_length = 0;
        turn_angle = 0;
        
        // Initialize random seed
        // randomSeed(analogRead(LIGHT_SENSOR_PINS[0]));
    }

    void setup() {
        Serial.begin(115200);
        delay(1000);
        current_state = SENSING;
        left_servo.attach(SERVO_PINS[0], minPulseWidth, maxPulseWidth);
        right_servo.attach(SERVO_PINS[1], minPulseWidth, maxPulseWidth);
    }

    void controlLoop() {
        switch (current_state) {
            case SENSING:
                light_angle = findLightDirection(); // determina is_occluded
                if (is_occluded) {
                    current_state = TURNING_TO_OBJ;
                } else {
                    current_state = PLANNING;
                }
                break;

            case PLANNING:
                step_length = levyStep();
                turn_angle = levyAngle();
                current_state = TURNING;
                break;

            case TURNING:
                // because turning will change where the light angle is 
                // PROBLEMAAA AQUI, SE SALE DEL RANGO -180 TO 180
                light_angle -= turn_angle; //  outside our range of angles, fix that
                while (!turnToAngle()) {
                    delay(10);
                }
                stop_servos();
                current_state = MOVING;
                break;

            case MOVING:
                // if we crash into something, the robot will stop in its 
                // tracks and not go the full levy step length
                while (!moveDistance()) { 
                    delay(10);
                }
                stop_servos();
                current_state = SENSING;
                break;
            
            case TURNING_TO_OBJ:
                turn_angle = light_angle; // turn to light
                while (!turnToAngle()) {
                    delay(10);
                }
                stop_servos();
                light_angle = turn_angle; // light is in front of you
                current_state = MOVING_TO_OBJ;
                break;
            
            case MOVING_TO_OBJ:
                step_length = MAX_STEPS_TO_FIND_OBJECT;
                // will move the max amount of steps until it hits a wall, hoping thats the object
                while (!moveDistance()) { 
                    delay(10);
                }
                stop_servos();
                current_state = PUSHING;
                break;
            
            case PUSHING:
                push();
                // delay(500);

                light_angle = findLightDirection(); // determina is_occluded
                if (!is_occluded) {
                    stop_servos();
                    current_state = PLANNING; // si ya hay luz, seguir buscando objeto
                }
                break;
        }
        updateStats();
    }

    // Public methods that might be useful for external control or monitoring
    bool getIsOccluded() const { return is_occluded; }
    bool getObjectDetected() const { return object_detected; }
    int getLightAngle() const { return light_angle; }
    int getStepLength() const { return step_length; }
    int getTurnAngle() const { return turn_angle; }

private:
    State current_state;
    Servo left_servo;
    Servo right_servo;
    
    // Constants
    static const int NUM_LIGHT_SENSORS = 4;
    static const int NUM_IR_SENSORS = 2;
    const int LIGHT_SENSOR_PINS[NUM_LIGHT_SENSORS] = {34, 35, 32, 33};
    const int IR_SENSOR_PINS[NUM_IR_SENSORS] = {5, 18};
    
    static constexpr int OCCLUDED_THRESHOLD = 1000;
    static const int MAX_STEPS_TO_FIND_OBJECT = 50;

    // servo parameters
    const int SERVO_PINS[2] = {23, 22}; // L R
    const int minPulseWidth = 500;
    const int maxPulseWidth = 2400;
    int move_proportional;
    const int max_CW = 180;
    const int max_CCW = 0;
    const int slow_down_by = 20;
    const int slow_CW = max_CW - slow_down_by;
    const int slow_CCW = max_CCW + slow_down_by;
    const int stop_angle = 90;
    int direction_object;
	// servo.write(0) makes it rotate counterclockwise at full speed.
    // servo.write(90) stops the 360-degree servo (neutral position).
	// servo.write(180) makes it rotate clockwise at full speed.
    
    // Lévy flight parameters
    static constexpr float alpha = 6.0;
    static constexpr float mu = 1.5;
    static constexpr float beta = 0.8;

    // State variables
    bool is_occluded;
    bool object_detected;
    int light_angle;
    int step_length;
    int turn_angle;

    const char* getStateName() {
        switch (current_state) {
            case SENSING: return "SENSING";
            case PLANNING: return "PLANNING";
            case TURNING: return "TURNING";
            case MOVING: return "MOVING";
            case TURNING_TO_OBJ: return "TURNING_TO_OBJ";
            case MOVING_TO_OBJ: return "MOVING_TO_OBJ";
            case PUSHING: return "PUSHING";
            default: return "UNKNOWN";
        }
    }
    
    // done
    int mapAngle(int angle) {
        if (angle > 90 && angle < 270) {
            return 90 - angle;
        } else {
            return (450 - angle) % 360;
        }
    }

    // done
    int findLightDirection() {
        int minLight = 4095;
        int minSensor = 0;

        for(int i = 0; i < NUM_LIGHT_SENSORS; i++) {
            int reading = analogRead(LIGHT_SENSOR_PINS[i]);
            if(reading < minLight) {
                minLight = reading;
                minSensor = i;
            }
        }

        if (minLight < OCCLUDED_THRESHOLD) {
            is_occluded = false;
            return mapAngle(minSensor * 90); // sees light, send that light direction
        } else {
            is_occluded = true;
            return light_angle; // if it's occluded, just remember the old direction
        }
    }


    // se usa dentro de una funcion, tiene variabl redundante
    bool isObstacleDetected() {
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            if (digitalRead(IR_SENSOR_PINS[i]) == LOW) {
                object_detected = true;
                return object_detected;
            }
        }
        object_detected = false;
        return object_detected;
    }

    int isAligned(){ // sensors go right to left
        int aligned_sensors = 0;
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            if (digitalRead(IR_SENSOR_PINS[i]) == LOW) {
                aligned_sensors += i+1;
            }
        }
        return aligned_sensors;
        // 0 no sensors
        // 1 right sensor
        // 2 left sensor
        // 3 both sensors
    }

    // last one, updating all stats
    void updateStats() {
        Serial.print("State: ");
        Serial.print(getStateName());  // Just use it directly
        
        Serial.print(" \t InShadow?: ");
        Serial.print(is_occluded);
        Serial.print(" \t HitaWall?: ");
        Serial.print(object_detected);
        Serial.print(" \t Light Angle: ");
        Serial.print(light_angle);
        Serial.print(" \t Step Error: ");
        Serial.print(step_length);
        Serial.print(" \t Angle Error: ");
        Serial.print(turn_angle);
        Serial.print(" \t direction_object: ");
        Serial.println(direction_object);
    }

    // revise
    int levyAngle() {
        int random_angle = (random(361) - 180);
        
        float diff = abs(random_angle - light_angle);
        if (diff > 180) {
            diff = 360 - diff;
        }
        
        if (diff < 60) {
            int opposite_light = (light_angle > 0) ? light_angle - 180 : light_angle + 180;
            int spread = 90;
            random_angle = opposite_light + (random(2 * spread + 1) - spread);
            
            if (random_angle > 180) random_angle -= 360;
            if (random_angle < -180) random_angle += 360;
        }
        
        return random_angle;
    }

    // done
    int levyStep() {
        float u = random(1000) / 1000.0;
        if (u == 0.0) u = 0.0001;
        
        float step = alpha * pow(u, -1.0 / mu);
        return (int)step;
    }

    // R CW +
    // L CCW -
    bool turnToAngle() {
        updateStats();  
        // proportional control
        move_proportional = turn_angle / 2; // makes the max value be +-90
        if (abs(turn_angle) > 0) {
            left_servo.write(stop_angle + move_proportional);  // CW
            right_servo.write(stop_angle + move_proportional); // CCW
            (turn_angle > 0) ? turn_angle-- : turn_angle++;
            delay(40);
            return false;
        } else {
            return true;
        }
    }

    // done
    bool moveDistance() {
        updateStats(); 
        move_proportional = 9*step_length; // proportional control
        if (step_length > 0) {
            if (isObstacleDetected()) {return true; }
            left_servo.write(stop_angle + move_proportional);  // CW
            right_servo.write(stop_angle - move_proportional); // CCW
            step_length--;
            delay(500); // normally you count the encoder here
            return false;
        } else {
            return true;
        } 
    }

    void push(){
        direction_object = isAligned();
        // 0 no sensors
        // 1 right sensor
        // 2 left sensor
        // 3 both sensors
        switch (direction_object) {
            case 0:
                // idk this should mean it lost the object, do nothing I guess
                break;
            case 1: // only right sensor, turn slight right
                left_servo.write(max_CW);
                right_servo.write(slow_CCW);
                break;
            case 2: // only left sensor, turn slight left
                left_servo.write(slow_CW);
                right_servo.write(max_CCW);
                break;
            case 3:
                left_servo.write(max_CW);
                right_servo.write(max_CCW);
                break;
        }
        delay(1000);
    }

    void stop_servos(){
        left_servo.write(stop_angle);
        right_servo.write(stop_angle);
    }
};