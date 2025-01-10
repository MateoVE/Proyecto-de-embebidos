#pragma once
#include <Arduino.h>            // general
#include <ESP32Servo.h>         // servo
#include <Wire.h>               // cable serial a compu
#include <Adafruit_GFX.h>       // display
#include <Adafruit_SSD1306.h>   // display
#include <ESP32Encoder.h>       // encoder
#include <apwifieeprommode.h>   // wifi de SE
#include <HTTPClient.h>         // Wifi
#include <Wifi.h>               // Wifi


class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), 
    integral(0), prev_error(100), prev_prev_error(0), time_in_SS(0) {}

    float compute(float target, float actual) {
        float error = target - actual;
        integral += error;
        float derivative = error - prev_error;
        prev_error = error;
        prev_prev_error = prev_error;
        Serial.println("prev error: " + String(prev_error));
        return Kp * error + Ki * integral + Kd * derivative;
    }

    bool reached_SS() {
        Serial.println(prev_error);
        if (time_in_SS >= 20) {
            time_in_SS = 0;
            return true;
        } else if (abs(prev_error - prev_prev_error) <= 10) { // 10 is the minimum tolerance 0.02 * max_error
            time_in_SS++;
        } else {
            time_in_SS = 0;
        }
        return false;
    }

private:
    float Kp, Ki, Kd;
    float integral, prev_error, prev_prev_error;
    int time_in_SS;
};


class PushingSwarmBot {
public:
    enum State {
        // SEARCHING mode
        PLANNING, 
        TURNING, 
        MOVING, 

        // PUSHING mode
        TURNING_TO_OBJ, 
        MOVING_TO_OBJ, 
        PUSHING_OBJ
    };

    enum Mode {
        SEARCHING,
        PUSHING
    };

    PushingSwarmBot() : // initializer
        
        // pid servos
        move_pid_left(1.0, 0, 0), 
        move_pid_right(1.0, 0, 0), 
        turn_pid_left(1.0, 0, 0), 
        turn_pid_right(1.0, 0, 0), 

        // display
        display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),

        // states
        current_mode(SEARCHING), current_state(PLANNING), 

        // state variables
        is_occluded(false), object_detected(false),
        light_angle(0), step_length(0), turn_angle(0) {}

    void setup() {
        Serial.begin(115200);
        delay(1000);

        // Configure pins
        for (int pin : LIGHT_SENSOR_PINS) pinMode(pin, INPUT);              // set light sensors as input
        for (int pin : IR_SENSOR_PINS) pinMode(pin, INPUT);                 // set IR sensors as input
        left_servo.attach(SERVO_PINS[0], minPulseWidth, maxPulseWidth);     // set up left servo
        right_servo.attach(SERVO_PINS[1], minPulseWidth, maxPulseWidth);    // set up right servo

        // Initialize encoders with pull-up resistors
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        
        // Attach encoders in full quadrature mode
        left_encoder.attachFullQuad(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
        right_encoder.attachFullQuad(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);
        
        // Clear encoder counts
        left_encoder.clearCount();
        right_encoder.clearCount();

        // Connect I2C comms
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

        // Initialize OLED display
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Serial.println("Failed to initialize OLED display");
            while (true); // Halt on error
        }

        display.clearDisplay();
        display.display();

        // Initialize wifi
        //intentoconexion("poto", "12341243"); // nombre de red del robot, clave
    }

    void controlLoop() {
        periodicSensing();
        
        // Simple P control for following light
        float p_gain = 0.1;  // Adjust this value to change turning sensitivity
        
        if (light_angle == 0) {
            // Go straight until obstacle detected
            if (!isObstacleDetected()) {
                left_servo.write(max_CW);
                right_servo.write(max_CCW);
            } else {
                stop_servos();
            }
        } else if (is_occluded) {
            stop_servos();
        } else {
            // Turn using proportional control
            int turn_speed = p_gain * light_angle;
            
            // Apply turning speeds to servos
            left_servo.write(constrain(stop_angle - turn_speed, max_CCW, max_CW));
            right_servo.write(constrain(stop_angle - turn_speed, max_CCW, max_CW));
        } 


    }
/*
    void controlLoop() {
        periodicSensing(); // Perform background sensing
        // we check for the direction of light so we always know where it is and if we went inside a shadow

        // High-level state machine
        switch (current_mode) {
            case SEARCHING:
                switch (current_state) {
                    case PLANNING:
                        planPath(); // Random path planning
                        transitionToState(TURNING);
                        break;

                    case TURNING:
                        if (turnToAngle(turn_angle)) {
                            // should update light angle since we turned? we are also checking light in the backgroun tho
                            // light angle - turn angle perhaps? check if in -180to180 range
                            transitionToState(MOVING);
                        }
                        break;

                    case MOVING:
                        if (moveDistance(step_length)) {
                            transitionToState(PLANNING); // Keep searching
                        }
                        break;
                }

                // If shadow is found during searching, switch to PUSHING mode
                if (is_occluded) {
                    transitionToMode(PUSHING);
                }
                break;

            case PUSHING:
                switch (current_state) {
                    case TURNING_TO_OBJ:
                        if (turnToAngle(light_angle)) {
                            transitionToState(MOVING_TO_OBJ);
                        }
                        break;

                    case MOVING_TO_OBJ:
                        if (moveDistance(MAX_STEPS_TO_FIND_OBJECT)) {
                            transitionToState(PUSHING_OBJ);
                        }
                        break;

                    case PUSHING_OBJ:
                        push(); // Push with alignment adjustments
                        // we push forever, until there is light. Add some fail safes here?
                        break;
                }

                // If light is found during searching, switch to PUSHING mode
                if (!is_occluded) {
                    transitionToMode(SEARCHING);
                }
                break;
        }
    }

*/

     void test_turn(int angle) {
        left_encoder.clearCount();
        right_encoder.clearCount();

        left_servo.write(constrain(stop_angle - 45, max_CCW, max_CW));
        right_servo.write(constrain(stop_angle - 45, max_CCW, max_CW));

        delay(250); // ms por grado
        stop_servos(); 

        Serial.println("turning");
        Serial.println(left_encoder.getCount());
        Serial.println(right_encoder.getCount());

        delay(1000);

    }

    void test_move(int distance) {
        left_encoder.clearCount();
        right_encoder.clearCount();

        left_servo.write(constrain(stop_angle + 90, max_CCW, max_CW));
        right_servo.write(constrain(stop_angle - 90, max_CCW, max_CW));

        delay(400); // 10cm en 250ms
        stop_servos(); 

        Serial.println("moving");
        Serial.println(left_encoder.getCount());
        Serial.println(right_encoder.getCount());

        delay(1000);


    }

private:
    // Objects
    Mode current_mode;
    State current_state;
    Servo left_servo, right_servo;
    PIDController move_pid_left, move_pid_right, turn_pid_left, turn_pid_right;
    Adafruit_SSD1306 display;
    ESP32Encoder left_encoder, right_encoder;

    // state variables
    bool is_occluded, object_detected;                  // determined by light and IR sensors respectively
    int light_angle;                                    // determined by light sensors
    int step_length, turn_angle;                        // levy random algorithm

    // PIN CONFIGURATIONS
        // light sensors
        static constexpr int NUM_LIGHT_SENSORS = 6;
        const int LIGHT_SENSOR_PINS[NUM_LIGHT_SENSORS] = {33, 32, 35, 34, 36, 39};
        const int SENSOR_ANGLES[NUM_LIGHT_SENSORS] = {-90, 180, 45, 90, -45, 0};      // light sensor angles index in order
        // IR sensors
        static constexpr int NUM_IR_SENSORS = 5;
        const int IR_SENSOR_PINS[NUM_IR_SENSORS] = {14, 15, 23, 5, 19};
    
        // servos
        const int SERVO_PINS[2] = {27, 18};
        const int LEFT_ENCODER_PINS[2] = {16, 4};
        const int RIGHT_ENCODER_PINS[2] = {12, 13};

    // LOGIC constants
    static constexpr int OCCLUDED_THRESHOLD = 300;
    static constexpr int MAX_STEPS_TO_FIND_OBJECT = 50;

    // Servo parameters
    static constexpr int minPulseWidth = 500, maxPulseWidth = 2500;
    static constexpr int stop_angle = 90;
    static constexpr int max_CW = 180, max_CCW = 0;

    // Display parameters
    static constexpr int SCREEN_WIDTH = 128;
    static constexpr int SCREEN_HEIGHT = 64;
    static constexpr int I2C_SDA_PIN = 21;
    static constexpr int I2C_SCL_PIN = 22;

    // Periodic sensing variables
    const int sensing_period_T = 500;
    unsigned long last_sensing_time = 0;

    // WIFI SHIT
    //const char *serverUrl = "https://webesp32.onrender.com/data"; // URL de tu servidor
    //String robotId = "Robot 0";

    // Functions
    const char* getStateName() {
        switch (current_state) {
            // display can only show 10 chars at a time horizontally
            case PLANNING: return "PLANNING";
            case TURNING: return "TURNING";
            case MOVING: return "MOVING";

            case TURNING_TO_OBJ: return "TURN->OBJ";
            case MOVING_TO_OBJ: return "MOVE->OBJ";
            case PUSHING_OBJ: return "PUSHING";
            default: return "UNKNOWN";
        }
    }

     const char* getModeName() {
        switch (current_mode) {
            case SEARCHING: return "SEARCHING";
            case PUSHING: return "PUSHING";
            default: return "UNKNOWN";
        }
    }

    void displayMessage(const char* mode, const char* state) {
        display.clearDisplay();
        display.setTextColor(WHITE);

        int textSize = 2;  // Scale factor for the text
        display.setTextSize(textSize);
        
        
        // mode
        int textWidth = strlen(mode) * 6 * textSize;
        int x = (SCREEN_WIDTH - textWidth) / 2;
        int y = 10;
        display.setCursor(x, y);
        display.println(mode);

        // state
        textWidth = strlen(state) * 6 * textSize;
        x = (SCREEN_WIDTH - textWidth) / 2;
        y = 40;
        display.setCursor(x, y);
        display.println(state);

        display.display(); // display it
    }

    void updateStats() {
        // Print in terminal
        Serial.print("Mode: ");
        Serial.print(getModeName());
        Serial.print("    State: ");
        Serial.print(getStateName());

        // Show on display
        displayMessage(getModeName(), getStateName());

        // Show on wifi
        //MsgOverWifi();
    }

    void transitionToState(State nextState) {
        current_state = nextState;          // state variable

        // reset encoders between each state
        left_encoder.clearCount();
        right_encoder.clearCount();

        stop_servos();                                   // and stop the servos

        // print in terminal for debugging
        Serial.print("Transitioning to state: ");
        Serial.println(getStateName());

        // we only update the display when we transition
        updateStats();
    }

    void transitionToMode(Mode next_mode) {
        current_mode = next_mode;       // state variable

        // Print in terminal for debugging
        Serial.println("Transitioning to MODE: ");
        Serial.print(getModeName());

        // Always transition to the first state of the corresponding mode
        switch (next_mode) {
            case SEARCHING:
                transitionToState(PLANNING);
                break;
            case PUSHING:
                transitionToState(TURNING_TO_OBJ);
            default:
                break;
        }
    }   

    void periodicSensing() {
        unsigned long now = millis();
        if (now - last_sensing_time > sensing_period_T) {    // Every 500 ms
            light_angle = findLightDirection(); // Update light direction
            last_sensing_time = now;            // Reset the timer
        }
    }
    
    void stop_servos() {
        left_servo.write(stop_angle);
        right_servo.write(stop_angle);
    }

    int findLightDirection() {
        int maxLight = 0; // Max light intensity detected, we start with the minimum
        int maxSensor; // Index of the sensor with max light

        // Iterate over all light sensors
        for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
            int reading = analogRead(LIGHT_SENSOR_PINS[i]); // Read sensor value
            if (reading > maxLight) { // we wanna find the sensor with the most light 
                maxLight = reading;
                maxSensor = i;
            }
        }

        // If no significant light is detected, mark as occluded
        if (maxLight <= OCCLUDED_THRESHOLD) {
            is_occluded = true;
            return light_angle; // Return the previous angle
        }

        is_occluded = false; // Light detected
        return SENSOR_ANGLES[maxSensor]; // Return the corresponding angle
    }

    bool turnToAngle(int target_angle) {
        bool both_in_steady_state = turn_pid_left.reached_SS() || turn_pid_right.reached_SS();
        if (!both_in_steady_state) { // verify 2% band
            Serial.println("turn?");
            
            // We calculate the control value for the servos
            int left_speed = turn_pid_left.compute(target_angle, right_encoder.getCount());
            int right_speed = turn_pid_right.compute(target_angle, right_encoder.getCount());

            // We write the speed we want and cap the value at the allowed bands 0-180 maxCCW and maxCW
            // servos should move in the same direction to turn, hence both speeds add
            left_servo.write(constrain(stop_angle + left_speed, max_CCW, max_CW));
            right_servo.write(constrain(stop_angle + right_speed, max_CCW, max_CW));

            return false; // we don't have 0 error so continue to turn
        } else {
            Serial.print("reached band");
            return true; // value reached 2% band so we reached our goal
        }
    }

    bool moveDistance(int target_distance) {
        bool both_in_steady_state = move_pid_left.reached_SS() || move_pid_right.reached_SS();
        if (!both_in_steady_state) { // verify 2% band
            // We calculate the control value for the servos
            int left_speed = move_pid_left.compute(target_distance, right_encoder.getCount());
            int right_speed = move_pid_right.compute(target_distance, right_encoder.getCount());

            if (isObstacleDetected()) {  // always check for collision and stop if there is one coming
                return true;
            }

            // We write the speed we want and cap the value at the allowed bands 0-180 maxCCW and maxCW
            // servos need to move in opposing directions to move forward, hence one speed is + the other -
            left_servo.write(constrain(stop_angle + left_speed, max_CCW, max_CW));
            right_servo.write(constrain(stop_angle - right_speed, max_CCW, max_CW));

            return false; // we don't have 0 error so continue to move
        } else {
            return true;  // value reached 2% band so we reached our goal
        }

        
    }
    
    bool isObstacleDetected() {
        for (int pin : IR_SENSOR_PINS) {                // going through every pin in the list IR_SENSOR_PINS
            if (digitalRead(pin) == LOW) {
                Serial.println("Obstacle");
                return true;   // as long as one detects an object, return true
            }
        }
        return false;                                   // if no sensors see anything, return false
    }

    void push() {
        // Read the two dedicated IR sensors (-10° and 10°)
        bool left_sensor_active = (digitalRead(IR_SENSOR_PINS[1]) == LOW);  // NW
        bool right_sensor_active = (digitalRead(IR_SENSOR_PINS[3]) == LOW); // NE

        const int slow_down = 20;
        if (!left_sensor_active && right_sensor_active) {
            // Only the right sensor detects the object: speed up lagging left side
            left_servo.write(max_CW);  // Faster on the left
            right_servo.write(max_CCW + slow_down); // Slower on the right
            Serial.println("Speeding up LEFT wheel.");
        } else if (left_sensor_active && !right_sensor_active) {
            // Only the left sensor detects the object: speed up lagging right side
            left_servo.write(max_CW - slow_down); // Slower on the left
            right_servo.write(max_CCW); // Faster on the right
            Serial.println("Speeding up RIGHT wheel.");
        } else {
            // Both sensors detect the object: push straight
            // Neither sensor detects the object: go forward to find it
            left_servo.write(max_CW);  // Full speed forward
            right_servo.write(max_CW); // Full speed forward
            Serial.println("Pushing straight.");
        }
    }

    void planPath() {
        Serial.println("Dist Steps: ");
        step_length = levyStep();    // random step
        Serial.print(step_length);

        Serial.println("Angle steps: ");
        turn_angle = levyAngle();    // random angle
        Serial.print(turn_angle);
    }

    int levyStep() {
        float u = random(1, 1000) / 1000.0;
        // step = alpha * pow(u, -1.0 / mu);
        return static_cast<int>(100 * pow(u, -1.0 / 1.5));
    }

    int levyAngle() {
        int ang = random(-180, 181);
        return ang*20; // random int between -180and180. Maybe add bias for a direction?
        
    }

/*
    void MsgOverWifi() {
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;

        // Especifica la URL del servidor
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Obtiene el estado como cadena
        const char *stateString = getStateName();

        // Construye el mensaje en formato JSON
        String jsonMessage = "{\"idRobot\": \"" + robotId + "\", \"estadoRobot\": \"" + stateString + "\"}";

        // Envía la solicitud POST
        int httpResponseCode = http.POST(jsonMessage);

        // Muestra la respuesta del servidor
        if (httpResponseCode > 0)
        {
        String response = http.getString();
        Serial.println("Respuesta del servidor: " + response);
        }
        else
        {
        Serial.println("Error en la solicitud: " + String(httpResponseCode));
        }

        // Finaliza la conexión HTTP
        http.end();
    }
    else
    {
        Serial.println("WiFi desconectado");
    }
    }
*/
};

