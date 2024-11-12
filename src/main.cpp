// tengo que medir estas señales y controlar estas salidas

// INPUT
// 8 distance sensors analog
// 4 light sensors analog

// 2 encoders?
// accelerometer 3 pins

// OUTPUT
// 2 servos PWM
// 1 display
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

// Define thresholds (adjust these based on testing)
#define IF_THRESHOLD 500
#define LIGHT_THRESHOLD 100
#define OCCLUDED_THRESHOLD 50
#define PI 3.141592653589793
#define NUM_SENSORS 8

// Se define los pines de los servos
#define servo1Pin 9             
#define servo2Pin 10            

// Array of pin numbers for each sensor
int sensor_pins[NUM_SENSORS] = {2, 3, 4, 5, 6, 7, 8, 9};

// Define parameters for Lévy flight
float alpha = 1.0;          // Scaling factor
float mu = 1.5;             // Power-law exponent
float beta = 0.8;           // Directional bias parameter

// Tuning parameters
int counts_per_degree = 5; // Number of encoder counts needed for a 1-degree turn (adjust based on testing)
int pwm_speed = 150;       // PWM speed for turning (0-255, adjust based on your motor and surface)
int wait = 100;            // 100 ms

// Function prototypes
int read_left_encoder();
int read_right_encoder();
void reset_encoders();
void set_left_motor_pwm(int speed);
void set_right_motor_pwm(int speed);
void move_forward();
void stop();
void reverse();
void turn();
void turn_to_goal();
void avoid_obstacle();
int read_IF_sensor(int sensor_id);
int read_photoresistor(int sensor_id);


// Global variables
int last_known_direction = 0; // Angle in degrees (0: front, 90: right, 180: back, 270: left)
bool is_occluded = false;
bool facing_goal = false;






// Placeholder functions for sensor readings
int read_IF_sensor(int sensor_id) {
    // Replace with actual sensor reading code
    return 0;
}






// Main obstacle avoidance function
void avoid_obstacle(float sensor_readings[]) {
    // -180 to 180
    int obstacle_angle = get_vector_reading(sensor_readings);

    // If no obstacles detected, return
    if (obstacle_angle == 360) {
        return;
    }

    // Determine turn direction (shortest path)
    int turn_direction = (obstacle_angle > 0) ? 1 : 0;  // 1 for right, 0 for left

    // Turn until either we've reached target angle or obstacle is no longer in front
    for (int i = 0; i < abs(obstacle_angle) && i < 360; i++) {  // Limit to 360 to prevent infinite loop
        // Turn one degree
        turn(turn_direction);

        // Get new vector reading and check if obstacle is still in front
        int new_obstacle_angle = get_vector_reading(sensor_readings);
        if (abs(new_obstacle_angle) > 100) { // if the obstacle is at least next to us, break
            break;  // Stop turning if obstacle is no longer in front
        }
    }
}











// ----------------------------- FUNCIONAMIENTO DE SERVOMOTORES MEDIANTE FUENTE DE LUZ (USANDO LIBRERIA ARDUINO.H)---------------------------
// Definicion de los pines de los sensores
// const int lightSensorPin1 = ..;
// const int lightSensorPin2 = ..;

int initialPosition = 90;  // Posición inicial de los servos en grados
int pulseMin = 544;        // Pulso mínimo (en microsegundos) para 0°
int pulseMax = 2400;       // Pulso máximo (en microsegundos) para 180°

// Función para ajustar el ángulo del servo mediante señal PWM
void setServoAngle(int pin, int angle) {
    // Calcular el ancho del pulso correspondiente al ángulo deseado
    int pulseWidth = map(angle, 0, 180, pulseMin, pulseMax);

    // Generar el pulso PWM en el pin del servo
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pin, LOW);

    // Completar el ciclo de 20 ms para mantener la frecuencia de 50 Hz
    delay(20 - (pulseWidth / 1000)); // Ajuste para que el ciclo completo sea 20 ms
}

void setup() {
    pinMode(servo1Pin, OUTPUT);
    pinMode(servo2Pin, OUTPUT);

    pinMode(lightSensorPin1, INPUT);
    pinMode(lightSensorPin2, INPUT);

    Serial.begin(9600);

    // Colocar servos en posición inicial
    setServoAngle(servo1Pin, initialPosition);
    setServoAngle(servo2Pin, initialPosition);
}

void loop() {
    // Leer valores de los sensores de luz
    int lightValue1 = analogRead(lightSensorPin1);
    int lightValue2 = analogRead(lightSensorPin2);

    // Verificar si la luz supera el umbral en ambos sensores
    if (lightValue1 > LIGHT_THRESHOLD && lightValue2 > LIGHT_THRESHOLD) {
        // Determinar dirección basada en valores de luz (ejemplo: movimiento hacia adelante)
        int movementDirection = map((lightValue1 + lightValue2) / 2, LIGHT_THRESHOLD, 1023, 0, 180);

        // Mover ambos servos en la dirección deseada
        setServoAngle(servo1Pin, movementDirection);
        setServoAngle(servo2Pin, movementDirection);
    } else {
        // Mantener servos en posición estática
        setServoAngle(servo1Pin, initialPosition);
        setServoAngle(servo2Pin, initialPosition);
    }

    delay(500);  // Intervalo de lectura
}

