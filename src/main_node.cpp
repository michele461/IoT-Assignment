#include <Arduino.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* 
#include <RadioLib.h>
#define LORA_CS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);

uint64_t joinEUI = 0x0000000000000000;
uint64_t devEUI  = 0x70B3D57ED0076AAB;
uint8_t appKey[] = { 0xCA, 0x31, 0x43, 0x61, 0xEE, 0x9F, 0xE0, 0xC8, 0x40, 0xAB, 0x49, 0x3E, 0x65, 0xAB, 0xD9, 0xF9 };
uint8_t nwkKey[] = { 0xCA, 0x31, 0x43, 0x61, 0xEE, 0x9F, 0xE0, 0xC8, 0x40, 0xAB, 0x49, 0x3E, 0x65, 0xAB, 0xD9, 0xF9 };
*/

#define INITIAL_SAMPLING_HZ 100
#define SAMPLES 128             
#define WINDOW_MS 5000          

#define SIGNAL_TYPE 1           
#define ENABLE_NOISE true
#define FILTER_TYPE 1           
#define NOISE_SIGMA 0.2
#define ANOMALY_PROB 0.02       

const char* ssid = "Vodafone-A37821247";
const char* password = "micheleclaudia";
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic = "sapienza/iot/test/average";

WiFiClient espClient;
PubSubClient mqtt(espClient);

struct SensorData {
    float rawValue;
    float filteredValue;
    bool isTrueAnomaly;      
    bool flaggedByFilter;    
    unsigned long timestamp; 
};

QueueHandle_t sampleQueue;
QueueHandle_t aggregateQueue;
volatile int currentDelayMs = 1000 / INITIAL_SAMPLING_HZ;

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, INITIAL_SAMPLING_HZ);

int truePositives = 0;
int falsePositives = 0;
int falseNegatives = 0;
int trueNegatives = 0;

float generateGaussianNoise(float mean, float stdDev) {
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    return mean + stdDev * sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

SensorData acquireSample(float t) {
    SensorData data;
    data.timestamp = millis();
    data.isTrueAnomaly = false;
    data.flaggedByFilter = false;

    float cleanSignal = 0;
    if (SIGNAL_TYPE == 1) cleanSignal = 2.0 * sin(2.0 * PI * 3.0 * t) + 4.0 * sin(2.0 * PI * 5.0 * t);
    else if (SIGNAL_TYPE == 2) cleanSignal = 3.0 * sin(2.0 * PI * 2.0 * t) + 1.5 * sin(2.0 * PI * 10.0 * t);
    else if (SIGNAL_TYPE == 3) cleanSignal = 5.0 * sin(2.0 * PI * 4.0 * t); 

    if (!ENABLE_NOISE) {
        data.rawValue = cleanSignal;
        data.filteredValue = cleanSignal;
        return data;
    }

    float noisySignal = cleanSignal + generateGaussianNoise(0.0, NOISE_SIGMA);

    if ((float)rand() / RAND_MAX < ANOMALY_PROB) {
        float spike = random(500, 1500) / 100.0; 
        if (random(0, 2) == 0) spike = -spike;
        noisySignal += spike;
        data.isTrueAnomaly = true;
    }

    data.rawValue = noisySignal;
    return data;
}

#define Z_HISTORY_SIZE 20
float zHistory[Z_HISTORY_SIZE];
int zIdx = 0;

void applyZScore(SensorData &d) {
    float sum = 0, mean = 0, variance = 0, stddev = 0;
    for(int i=0; i<Z_HISTORY_SIZE; i++) sum += zHistory[i];
    mean = sum / Z_HISTORY_SIZE;
    
    for(int i=0; i<Z_HISTORY_SIZE; i++) variance += pow(zHistory[i] - mean, 2);
    stddev = sqrt(variance / Z_HISTORY_SIZE);

    if (stddev > 0.1) {
        float z = abs((d.rawValue - mean) / stddev);
        if (z > 3.0) d.flaggedByFilter = true;
    }

    if (d.flaggedByFilter) d.filteredValue = mean; 
    else d.filteredValue = d.rawValue;

    zHistory[zIdx] = d.filteredValue;
    zIdx = (zIdx + 1) % Z_HISTORY_SIZE;
}

#define H_WINDOW 7
float hBuffer[H_WINDOW];
int hIdx = 0;

void applyHampel(SensorData &d) {
    hBuffer[hIdx] = d.rawValue;
    hIdx = (hIdx + 1) % H_WINDOW;

    float temp[H_WINDOW];
    memcpy(temp, hBuffer, sizeof(hBuffer));
    for(int i=0; i<H_WINDOW-1; i++) {
        for(int j=i+1; j<H_WINDOW; j++) {
            if(temp[i] > temp[j]) { float t = temp[i]; temp[i] = temp[j]; temp[j] = t; }
        }
    }
    float median = temp[H_WINDOW / 2];

    float deviations[H_WINDOW];
    for(int i=0; i<H_WINDOW; i++) deviations[i] = abs(hBuffer[i] - median);
    for(int i=0; i<H_WINDOW-1; i++) {
        for(int j=i+1; j<H_WINDOW; j++) {
            if(deviations[i] > deviations[j]) { float t = deviations[i]; deviations[i] = deviations[j]; deviations[j] = t; }
        }
    }
    float mad = deviations[H_WINDOW / 2];

    if (abs(d.rawValue - median) > 3.0 * 1.4826 * mad) { 
        d.flaggedByFilter = true;
        d.filteredValue = median; 
    } else {
        d.filteredValue = d.rawValue;
    }
}

void TaskSample(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        float t = millis() / 1000.0;
        SensorData data = acquireSample(t);
        xQueueSend(sampleQueue, &data, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(currentDelayMs));
    }
}

void TaskAnalyze(void *pvParameters) {
    int sampleCount = 0;
    float windowSum = 0;
    int windowCount = 0;
    unsigned long windowStartTime = millis();
    bool adaptiveDone = false;

    for (;;) {
        SensorData d;
        if (xQueueReceive(sampleQueue, &d, portMAX_DELAY) == pdPASS) {
            unsigned long filterStartTime = esp_timer_get_time();
            
            if (FILTER_TYPE == 1) applyZScore(d);
            else if (FILTER_TYPE == 2) applyHampel(d);
            else d.filteredValue = d.rawValue;

            unsigned long filterExecTime = esp_timer_get_time() - filterStartTime;

            if (ENABLE_NOISE && FILTER_TYPE != 0) {
                if (d.isTrueAnomaly && d.flaggedByFilter) truePositives++;
                else if (!d.isTrueAnomaly && d.flaggedByFilter) falsePositives++;
                else if (d.isTrueAnomaly && !d.flaggedByFilter) falseNegatives++;
                else if (!d.isTrueAnomaly && !d.flaggedByFilter) trueNegatives++;
            }

            windowSum += d.filteredValue;
            windowCount++;

            if (!adaptiveDone && sampleCount < SAMPLES) {
                vReal[sampleCount] = d.filteredValue;
                vImag[sampleCount] = 0.0;
                sampleCount++;
                
                if (sampleCount == SAMPLES) {
                    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
                    FFT.compute(FFTDirection::Forward);
                    FFT.complexToMagnitude();

                    double f_max = 0;
                    for (int i = (SAMPLES / 2) - 1; i > 0; i--) {
                        if (vReal[i] > 20.0) { 
                            f_max = (i * 1.0 * INITIAL_SAMPLING_HZ) / SAMPLES;
                            break;
                        }
                    }
                    
                    int newFreq = (int)(f_max * 2.0) + 2; 
                    if(newFreq < 5) newFreq = 5;
                    currentDelayMs = 1000 / newFreq;
                    
                    Serial0.printf("\n[FFT] F_max found: %.2f Hz\n", f_max);
                    Serial0.printf("[ADAPT] New Freq: %d Hz -> Delay changed from 10ms to %dms\n", newFreq, currentDelayMs);
                    adaptiveDone = true;
                }
            }

            if (millis() - windowStartTime >= WINDOW_MS) {
                if (windowCount > 0) {
                    SensorData aggData;
                    aggData.filteredValue = windowSum / windowCount;
                    aggData.timestamp = d.timestamp; 

                    Serial0.println("\n=== 5s WINDOW REPORT ===");
                    Serial0.printf("Aggregated Mean: %.2f\n", aggData.filteredValue);
                    Serial0.printf("Filter Exec Time: %lu us per sample\n", filterExecTime);
                    
                    if (ENABLE_NOISE && (truePositives+falseNegatives)>0) {
                        float TPR = (float)truePositives / (truePositives + falseNegatives);
                        float FPR = (float)falsePositives / (falsePositives + trueNegatives);
                        Serial0.printf("[FILTER EVAL] TPR: %.2f%% | FPR: %.2f%%\n", TPR*100, FPR*100);
                    }

                    xQueueSend(aggregateQueue, &aggData, 0); 
                }
                windowSum = 0;
                windowCount = 0;
                windowStartTime = millis();
            }
        }
    }
}

void TaskTransmit(void *pvParameters) {
    mqtt.setServer(mqtt_server, 1883);
    /* RIMOVI QUESTO COMMENTO PER ATTIVARE LORAWAN
    int loraDutyCycle = 0;
    */
    for (;;) {
        if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
            mqtt.connect("ESP32_Test_Edge");
        }
        mqtt.loop(); 

        SensorData agg;
        if (xQueueReceive(aggregateQueue, &agg, portMAX_DELAY) == pdPASS) {
            String payload = String(agg.filteredValue, 2);
            
            if (mqtt.connected()) {
                mqtt.publish(mqtt_topic, payload.c_str());
                unsigned long latency = millis() - agg.timestamp;
                Serial0.printf("[TX-WIFI] Sent: %s | Payload: %d bytes | E2E Latency: %lu ms\n", 
                              payload.c_str(), payload.length() + 30, latency); 
            }

            /*
            loraDutyCycle++;
            if (loraDutyCycle >= 6) { 
                Serial0.printf("[TX-LORA] Sending to TTN: %s | Bytes: %d\n", payload.c_str(), payload.length());
                int16_t state = node.sendReceive((uint8_t*)payload.c_str(), payload.length());
                if (state == RADIOLIB_ERR_NONE) Serial0.println("[LORA] Success!");
                loraDutyCycle = 0; 
            }
            */
        }
    }
}

void setup() {
    Serial0.begin(115200);
    
    unsigned long startTime = millis();
    while (!Serial0 && (millis() - startTime) < 5000) {
        delay(10); 
    }
    delay(1000); 
    
    Serial0.println("\n\n====================================");
    Serial0.println("SYSTEM BOOT! Serial connection established.");
    Serial0.println("====================================\n");

    /*
    Serial0.println("[LORA] Starting...");
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
        state = node.activateOTAA();
        if (state == RADIOLIB_ERR_NONE) Serial0.println("[LORA] TTN Join Success!");
    }
    */

    WiFi.begin(ssid, password);
    
    for(int i=0; i<Z_HISTORY_SIZE; i++) zHistory[i] = 0;
    for(int i=0; i<H_WINDOW; i++) hBuffer[i] = 0;

    sampleQueue = xQueueCreate(256, sizeof(SensorData));
    aggregateQueue = xQueueCreate(10, sizeof(SensorData));

    xTaskCreatePinnedToCore(TaskSample, "Samp", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TaskAnalyze, "Anal", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskTransmit, "Tx", 4096, NULL, 1, NULL, 1);
}

void loop() { 
    vTaskDelete(NULL); 
}