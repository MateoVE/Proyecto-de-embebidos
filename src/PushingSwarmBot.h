#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>

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
                while (!turnToAngle()) {
                    delay(10);
                }
                current_state = MOVING;
                break;

            case MOVING:
                // if we crash into something, the robot will stop in its 
                // tracks and not go the full levy step length
                while (!moveDistance()) { 
                    delay(10);
                }
                current_state = SENSING;
                break;
            
            case TURNING_TO_OBJ:
                turn_angle = light_angle; // turn to light
                while (!turnToAngle()) {
                    delay(10);
                }
                light_angle = turn_angle; // light is in front of you
                current_state = MOVING_TO_OBJ;
                break;
            
            case MOVING_TO_OBJ:
                step_length = MAX_STEPS_TO_FIND_OBJECT;
                // will move the max amount of steps until it hits a wall, hoping thats the object
                while (!moveDistance()) { 
                    delay(10);
                }
                current_state = PUSHING;
                break;
            
            case PUSHING:
                push();
                delay(500);

                light_angle = findLightDirection(); // determina is_occluded
                if (!is_occluded) current_state = PLANNING; // si ya hay luz, seguir buscando objeto
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
    
    // Constants
    static const int NUM_LIGHT_SENSORS = 4;
    static const int NUM_IR_SENSORS = 2;
    const int LIGHT_SENSOR_PINS[NUM_LIGHT_SENSORS] = {34, 35, 32, 33};
    const int IR_SENSOR_PINS[NUM_IR_SENSORS] = {5, 18};
    static constexpr int OCCLUDED_THRESHOLD = 1000;
    static const int MAX_STEPS_TO_FIND_OBJECT = 50;
    
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

    // last one, updating all stats
    void updateStats() {
        Serial.print("State: ");
        Serial.print(getStateName());  // Just use it directly
        
        Serial.print(" \t Shadow: ");
        Serial.print(is_occluded);
        Serial.print(" \t Obstruction: ");
        Serial.print(object_detected);
        Serial.print(" \t Light Ang: ");
        Serial.print(light_angle);
        Serial.print(" \t Step Er: ");
        Serial.print(step_length);
        Serial.print(" \t Angle Er: ");
        Serial.println(turn_angle);
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

    // done
    bool turnToAngle() {
        updateStats();  
        if (abs(turn_angle) > 0) {
            delay(40);
            (turn_angle > 0) ? turn_angle-- : turn_angle++;
            return false;
        } else {
            return true;
        }
    }

    // done
    bool moveDistance() {
        updateStats(); 
        if (step_length > 0) {
            if (isObstacleDetected()) { return true; }
            step_length--;
            delay(500);
            return false;
        } else {
            return true;
        } 
    }

    void push(){
        delay(10);
    }
};