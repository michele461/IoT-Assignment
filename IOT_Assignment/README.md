# IoT Edge Computing & Adaptive Sampling System

## 1. Project Overview
This repository contains the firmware and evaluation for an advanced IoT edge device based on FreeRTOS. The system collects data from a virtual noisy sensor, mitigates anomalies using Edge Computing filters (Z-Score/Hampel), dynamically adapts its sampling frequency using Fast Fourier Transform (FFT) to save energy, and transmits aggregated data via a hybrid network (MQTT over Wi-Fi for Edge, LoRaWAN via TTN for Cloud).

## 2. System Architecture & FreeRTOS
The firmware runs on an ESP32-S3 (Heltec V3) and is structured into 3 parallel FreeRTOS tasks to ensure real-time execution without blocking:
* **TaskSample (Producer):** Acquires data at high frequency (100Hz default) and utilizes `vTaskDelayUntil()` to trigger the microcontroller's light sleep between readings.
* **TaskAnalyze (Edge Computing):** Filters noise, computes the FFT to find the dominant frequency, and calculates a 5-second window average.
* **TaskTransmit (Publisher):** Sends the aggregated average to the Edge (MQTT) every 5 seconds and to the Cloud (LoRaWAN) every ~30 seconds to respect duty cycle constraints.

## 3. Core Performance Evaluation

### Energy Savings (Measured via external INA219)
We measured the power consumption using a second ESP32 acting as an I2C power monitor.
* **Baseline (100 Hz Oversampling):** The CPU is constantly active. Average consumption: `[INSERISCI I TUOI MW QUI]` mW.
* **Adaptive Sampling (12 Hz via FFT):** Following Nyquist's theorem based on the FFT dominant frequency (5 Hz), the sampling rate is reduced, allowing FreeRTOS to maximize Light Sleep. Average consumption drops to: `[INSERISCI I TUOI MW QUI]` mW.

### Latency and Execution Time
* **Filter Execution Time:** The anomaly filter requires approx `[INSERISCI US]` µs per sample.
* **End-to-End Latency:** The delay from data generation to MQTT edge reception is on average `[INSERISCI MS]` ms.
* **Data Volume:** Transmitting 5-second averages instead of raw 100Hz data reduces the payload to just `~4 bytes` per window, dramatically lowering communication overhead.

## 4. Bonus Evaluation: Noise, Anomalies & Filtering

### Input Signals & Adaptive FFT
We tested 3 different mathematical signals. 
* **Clean Signal (3Hz + 5Hz):** FFT successfully identified ~5Hz, lowering the rate to 12Hz.
* *Note on Anomalies:* When Gaussian noise (σ=0.2) and large synthetic spikes (p=0.02) are injected, the raw FFT detects high-frequency noise up to 49Hz, preventing energy savings. Pre-filtering is essential.

### Z-Score vs Hampel Filter Comparison
We implemented an anomaly-aware pipeline to clean the signal before FFT and aggregation.
* **Z-Score:** Very fast (`~2 µs`), but sensitive to extreme outliers skewing the mean. Achieved True Positive Rate (TPR): `~65%`, False Positive Rate (FPR): `~9%`.
* **Hampel (Median based):** More computationally expensive (`~[INSERISCI US] µs` due to array sorting) but robust against large spikes. Achieved TPR: `[INSERISCI %]`, FPR: `[INSERISCI %]`.

## 5. Hardware Setup
* **Node:** Heltec WiFi LoRa 32 V3 (Target device)
* **Monitor:** ESP32 + INA219 Current Sensor
* **Wiring:** INA219 placed in series between the 5V supply and the Heltec's 5V input. I2C connected to the Monitor ESP32.

## 6. LLM Usage & Prompts
During this project, Gemini was used as a pair-programming ally to optimize FreeRTOS queue management, integrate the Box-Muller transform for Gaussian noise, and design the INA219 dual-ESP32 architecture.

*Sample Prompts used:*
1.  "How to properly format an ESP32-S3 platformio.ini to enable USB CDC Serial monitoring?"
2.  "Implement a FreeRTOS Task to calculate Hampel filter and Z-score over a rolling window..."

*Limitations & Opportunities:* The LLM was incredibly fast at generating boilerplate code and math algorithms (like FFT integration). However, it initially struggled with the hardware-specific quirks of the ESP32-S3 USB routing, requiring manual debugging and specific prompt constraints to resolve `Serial0` vs `Serial` issues.
