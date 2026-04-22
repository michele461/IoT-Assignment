#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define SDA_PIN 41
#define SCL_PIN 42

Adafruit_INA219 ina219;

float powerSum = 0;
float currentSum = 0; 
int readings = 0;
unsigned long lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n--- POWER MONITOR START ---");
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!ina219.begin(&Wire)) {
        Serial.println("Failed to find INA219 chip. Check wiring!");
        while (1) { delay(10); }
    }
    Serial.println("INA219 Connected.");
    lastPrintTime = millis();
}

void loop() {
    float power_mW = ina219.getPower_mW();
    float current_mA = ina219.getCurrent_mA();

    powerSum += power_mW;
    currentSum += current_mA;
    readings++;

    if (millis() - lastPrintTime >= 5000) {
        float avgPower = powerSum / readings;
        float avgCurrent = currentSum / readings; 
        Serial.println("=============================");
        Serial.printf("Average Power   (Last 5s): %.2f mW\n", avgPower);
        Serial.printf("Average Current (Last 5s): %.2f mA\n", avgCurrent);
        Serial.println("=============================\n");
        powerSum = 0;
        currentSum = 0;
        readings = 0;
        lastPrintTime = millis();
    }
    
    delay(10); 
}