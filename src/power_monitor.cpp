#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

float powerSum = 0;
int readings = 0;
unsigned long lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n--- POWER MONITOR START ---");

    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip. Check wiring!");
        while (1) { delay(10); }
    }
    Serial.println("INA219 Connected.");
    lastPrintTime = millis();
}

void loop() {
    float power_mW = ina219.getPower_mW();
    powerSum += power_mW;
    readings++;

    if (millis() - lastPrintTime >= 5000) {
        float avgPower = powerSum / readings;
        Serial.println("=============================");
        Serial.printf("Average Power (Last 5s): %.2f mW\n", avgPower);
        Serial.println("=============================\n");
        
        powerSum = 0;
        readings = 0;
        lastPrintTime = millis();
    }
    
    delay(10); 
}