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


class Tests {
public:
    Tests()
        : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1) {
    }

    void setup() {
        Serial.begin(115200);  // Increased baud rate for faster serial output
        delay(1000);

        // Configure pins
        for (int pin : LIGHT_SENSOR_PINS) pinMode(pin, INPUT);
        for (int pin : IR_SENSOR_PINS) pinMode(pin, INPUT);
        for (int pin : SERVO_PINS) pinMode(pin, OUTPUT);

        // Initialize servos with specified pulse widths
        left_servo.attach(SERVO_PINS[0], minPulseWidth, maxPulseWidth);
        right_servo.attach(SERVO_PINS[1], minPulseWidth, maxPulseWidth);

        // Initialize encoders with pull-up resistors
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        
        // Attach encoders in full quadrature mode
        left_encoder.attachFullQuad(LEFT_ENCODER_PINS[0], LEFT_ENCODER_PINS[1]);
        right_encoder.attachFullQuad(RIGHT_ENCODER_PINS[0], RIGHT_ENCODER_PINS[1]);
        
        // Clear encoder counts
        left_encoder.clearCount();
        right_encoder.clearCount();

        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

        // Initialize OLED display
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Serial.println("Failed to initialize OLED display");
            while (true); // Halt on error
        }

        display.clearDisplay();
        display.display();

        // Initialize wifi
        intentoconexion("poto", "12341243");
    }

    void testOled() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.println("Hello World");
        display.display();
    }

    // Test functions for individual components
    void testLight() {
        Serial.println("Light Sensor Readings:");
        for (int i = 0; i < NUM_LIGHT_SENSORS; ++i) {
            Serial.printf("Sensor %d: %d\n", i, analogRead(LIGHT_SENSOR_PINS[i]));
        }
    }

    void testDist() {
        Serial.println("IR Sensor Readings:");
        for (int i = 0; i < NUM_IR_SENSORS; ++i) {
            Serial.printf("Sensor %d: %d\n", i, digitalRead(IR_SENSOR_PINS[i]));
        }
    }

    void testServos(){
        left_servo.write(180);
        right_servo.write(180);
    }

    void stopServos(){
        left_servo.write(90);
        right_servo.write(90);
    }


    void monitorEncoders() {
        // Reset encoders to start from zero
        
        Serial.println("Monitoring encoders. Format: Left Count, Right Count");
        Serial.println("Left,Right");  // CSV header
        
        // Print initial values
        Serial.printf("%lld,%lld\n", 
            left_encoder.getCount(),
            right_encoder.getCount()
        );
    }

    void MsgOverWifi()
    {
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;

        // Especifica la URL del servidor
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Obtiene el estado como cadena
        const char *stateString = "Pipi";
        String robotId = "Poto";

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
private:
    // Hardware objects
    Servo left_servo, right_servo;
    Adafruit_SSD1306 display;
    ESP32Encoder left_encoder, right_encoder;

    // Pin configurations
    static constexpr int NUM_LIGHT_SENSORS = 8;
    const int LIGHT_SENSOR_PINS[NUM_LIGHT_SENSORS] = {33, 32, 35, 34, 36, 39};
    static constexpr int NUM_IR_SENSORS = 5;
    const int IR_SENSOR_PINS[NUM_IR_SENSORS] = {14, 15, 5, 19, 23};
    const int SERVO_PINS[2] = {25, 18};
    const int LEFT_ENCODER_PINS[2] = {16, 4};
    const int RIGHT_ENCODER_PINS[2] = {12, 13};

    // Hardware parameters
    static constexpr int minPulseWidth = 500;
    static constexpr int maxPulseWidth = 2400;
    static constexpr int SCREEN_WIDTH = 128;
    static constexpr int SCREEN_HEIGHT = 64;
    static constexpr int I2C_SDA_PIN = 21;
    static constexpr int I2C_SCL_PIN = 22;

    // WIFI SHIT
    const char *serverUrl = "https://webesp32.onrender.com/data"; // URL de tu servidor
    
};