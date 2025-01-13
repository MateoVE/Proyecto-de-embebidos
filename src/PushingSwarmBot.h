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


class PushingSwarmBot {
public:
    enum State {
        SEARCHING_FOR_SHADOW,    // Looking for light
        TURNING_TO_LIGHT,      // Turning towards light or random direction
        MOVING_TO_BOX,
        PUSHING_BOX      // Pushing box when occluded
    };

    PushingSwarmBot() : 
        display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
        current_state(SEARCHING_FOR_SHADOW),
        is_occluded(false),
        light_angle(0),
        last_state_change(0),
        turn_duration(0) {}


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
        // left_encoder.attachFullQuad(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
        // right_encoder.attachFullQuad(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);
        
        // Clear encoder counts
        // left_encoder.clearCount();
        // right_encoder.clearCount();

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
        
        switch (current_state) {
            case SEARCHING_FOR_SHADOW:
                if (is_occluded) {
                    // Found a shadow! Remember where light was and turn that way
                    transitionToState(TURNING_TO_LIGHT);
                } else {
                    // No shadow yet, keep doing random walk
                    randomWalk();
                }
                break;

            case TURNING_TO_LIGHT:
                if (millis() - last_state_change < turn_duration) {
                    // Keep turning towards last known light position
                    turn(light_angle > 0);
                } else {
                    // Done turning, now move forward to find box
                    light_angle = 0; // we assume we turned enough so light is in front of us now
                    transitionToState(MOVING_TO_BOX);
                }
                break;

            case MOVING_TO_BOX:
                if (isObstacleDetected()) {
                    if (is_occluded) { // Hit something while in shadow - must be the box
                        transitionToState(PUSHING_BOX);
                    } else { // Hit something in the light - must be a wall
                        transitionToState(SEARCHING_FOR_SHADOW);
                    }
                } else {
                    moveForward();
                }
                break;

            case PUSHING_BOX:
                if (!is_occluded) {
                    // If we see light again, go back to searching
                    transitionToState(SEARCHING_FOR_SHADOW);
                } else {
                    pushWithAlignment();
                }
                break;
        }

        updateDisplay();
    }

    void test_turn(int angle) {


        left_servo.write(stop_angle - 45);
        right_servo.write(stop_angle - 45);

        delay(250); // ms por grado
        stop_servos(); 

        Serial.println("turning");


        delay(1000);

    }

    void test_move(int distance) {


        left_servo.write(stop_angle + 90);
        right_servo.write(stop_angle - 90);

        delay(400); // 10cm en 250ms
        stop_servos(); 

        Serial.println("moving");


        delay(1000);
    }


private:
    Adafruit_SSD1306 display;
    State current_state;
    Servo left_servo, right_servo;
    unsigned long last_state_change;
    unsigned long turn_duration;

    // state variables
    bool is_occluded, object_detected;                  // determined by light and IR sensors respectively
    int light_angle;                                    // determined by light sensors

    // PIN CONFIGURATIONS
        // light sensors
        static constexpr int NUM_LIGHT_SENSORS = 6;
        const int LIGHT_SENSOR_PINS[NUM_LIGHT_SENSORS] = {33, 32, 35, 34, 36, 39};
        const int SENSOR_ANGLES[NUM_LIGHT_SENSORS] = {-90, 180, 45, 90, -45, 0};      // light sensor angles index in order
        // IR sensors
        static constexpr int NUM_IR_SENSORS = 5;
        const int IR_SENSOR_PINS[NUM_IR_SENSORS] = {14, 15, 23, 5, 19};
    
        // servos
        const int SERVO_PINS[2] = {18, 27};  // 27, 18 normal
        const int LEFT_ENCODER_PINS[2] = {16, 4};
        const int RIGHT_ENCODER_PINS[2] = {12, 13};

    // LOGIC constants
    static constexpr int OCCLUDED_THRESHOLD = 50;

    // Servo parameters
    static constexpr int minPulseWidth = 500, maxPulseWidth = 2500;
    static constexpr int stop_angle = 90;
    static constexpr int max_CW = 120, max_CCW = 60; // its actually 180 and 0
    static constexpr int turn_speed = 30;
    static constexpr int align_speed = 20; // Speed differential for alignment

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
            case SEARCHING_FOR_SHADOW: return "SEARCHING";
            case TURNING_TO_LIGHT: return "TURNING";
            case MOVING_TO_BOX: return "MOVING";
            case PUSHING_BOX: return "PUSHING";
            default: return "UNKNOWN";
        }
    }

    void transitionToState(State new_state) {
        stop_servos();
        current_state = new_state;
        last_state_change = millis();
        
        // Set turn duration if transitioning to TURNING
        if (new_state == TURNING_TO_LIGHT) {
            if (current_state == SEARCHING_FOR_SHADOW) {
                turn_duration = random(300, 1000); // Random search turn
            } else {
                turn_duration = map(abs(light_angle), 0, 180, 100, 1000); // Proportional to angle
            }
        }
    }

    void periodicSensing() {
        unsigned long now = millis();
        if (now - last_sensing_time > sensing_period_T) {    // Every 500 ms
            light_angle = findLightDirection(); // Update light direction
            last_sensing_time = now;            // Reset the timer
        }
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

    bool isObstacleDetected() {
        for (int pin : IR_SENSOR_PINS) {                // going through every pin in the list IR_SENSOR_PINS
            if (digitalRead(pin) == LOW) {
                return true;   // as long as one detects an object, return true
            }
        }
        return false;                                   // if no sensors see anything, return false
    }

    void moveForward() {
        left_servo.write(max_CW);
        right_servo.write(max_CCW);
    }

    void turn(bool clockwise) {
        if (clockwise) {
            left_servo.write(stop_angle - turn_speed); // should be positive but only works with negative
            right_servo.write(stop_angle - turn_speed);
        } else {
            left_servo.write(stop_angle + turn_speed);
            right_servo.write(stop_angle + turn_speed);
        }
    }

    void pushWithAlignment() {
        // Use only front sensors for alignment
        bool nw_active = (digitalRead(IR_SENSOR_PINS[1]) == LOW); // NW sensor
        bool n_active = (digitalRead(IR_SENSOR_PINS[2]) == LOW);  // N sensor
        bool ne_active = (digitalRead(IR_SENSOR_PINS[3]) == LOW); // NE sensor

        if (!n_active) {
            // Lost the box, go back to searching
            transitionToState(SEARCHING_FOR_SHADOW);
        } else if (nw_active && !ne_active) {
            // Box is to the left, turn left slightly
            left_servo.write(max_CW - align_speed);
            right_servo.write(max_CCW);
        } else if (!nw_active && ne_active) {
            // Box is to the right, turn right slightly
            left_servo.write(max_CW);
            right_servo.write(max_CCW - align_speed);
        } else {
            // Centered on box, push straight
            moveForward();
        }
    }

    void randomWalk() {
        // Every few seconds, pick a new random direction
        if (millis() - last_state_change > 2000) {
            stop_servos();
            delay(100);
            turn(random(2));  // Pick random direction
            delay(random(100, 600));  // Turn for random time
            last_state_change = millis();
        }
        if (!isObstacleDetected()){
            moveForward();  // Keep moving forward
        }
        
    }

    void updateDisplay() {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,0);

        display.println(getStateName()); 
        
        display.print("Luz: ");
        display.println(is_occluded ? "NO" : "SI");
        
        display.print("Ang: ");
        display.println(light_angle);

        display.print("Obs: ");
        display.println(isObstacleDetected() ? "SI" : "NO");
        
        display.display();
    }

    void stop_servos() {
        left_servo.write(stop_angle);
        right_servo.write(stop_angle);
    }

/*
    void MsgOverWifi() {
        if (WiFi.status() == WL_CONNECTED) {
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
            if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("Respuesta del servidor: " + response);
            } else {
            Serial.println("Error en la solicitud: " + String(httpResponseCode));
            }

            // Finaliza la conexión HTTP
            http.end();
        } else {
            Serial.println("WiFi desconectado");
        }
    }    
*/

};