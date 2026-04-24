# IoT Individual Assignment - Michele Libriani 1954541

## 1. Project Overview
In this repository, you will find all the necessary files for a complete IoT system developed using **FreeRTOS** that collects data from a sensor, analyzes it locally (Edge Computing), adapts its sampling frequency to save energy, and transmits the aggregated values as reports to both an MQTT Edge Server via Wi-Fi and to The Things Network (TTN) Cloud via LoRaWAN.

The hardware setup is based on a "node-monitor" architecture, structured with a Heltec WiFi LoRa ESP32 V3 as the target node (main) and a second Heltec WiFi LoRa ESP32 V3 combined with an INA219 current sensor for power monitoring. The INA219 is placed in series between the power supply (battery/5V) and the Node's 5V input. Its I2C pins (SDA, SCL) are connected to the Monitor ESP32.

### Folder Structure
The project follows the standard PlatformIO structure:
* `.pio/` and `.vscode/`: Build and development environment configuration folders.
* `src/`: Contains the source code (`main_node.cpp` and `power_monitor.cpp`).
* `logs/`: Folder containing all log files extracted from the tests (e.g., `rm1.txt` for the MQTT/LoRa reports of Run Mode 1, `rm1_ec.txt` for the Energy Consumption measurements of Run Mode 1, and so on for all 5 modes).
* `.gitignore`: hide .vscode and .pio repositories.
* `platformio.ini`: contains all the libraries and specifics for both the two environments (main_node and power_monitor).
* `README.md`

# 2. Code Architecture

### Node (`src/main_node.cpp`)
The code runs on the target ESP32-S3 and is divided into 3 FreeRTOS tasks.

**Global Variables and Directives:**
* `#define RUN_MODE`: select which test to execute.
* Constants (`INITIAL_SAMPLING_HZ 100`, `SAMPLES 128`, `WINDOW_MS 5000`).
* **`volatile` variables** (`SIGNAL_TYPE`, `FILTER_TYPE`, `ANOMALY_PROB`, `DYNAMIC_WINDOW`): modified by `TaskAnalyze` during phase transitions and read by `TaskSample`.

**Data Structures and Functions:**
* `SensorData`: struct passed through FreeRTOS queues containing raw, filtered, and clean values, anomaly flags, and timestamps for latency.
* `generateGaussianNoise()`: uses the Box-Muller transform to generate realistic Gaussian noise.
* `acquireSample(t)`: generates the clean signal, adding optional noise and artificial anomaly spikes.
* `applyZScore()`: flags values deviating >3 standard deviations from the mean.
* `applyHampel()`: uses median and MAD for robust outlier detection, sorting the array dynamically.
* `setupInitialState()` and `updatePhaseState()`: state machine handling automatic transitions of signals, window sizes, and anomaly probabilities.

**Tasks and System Flow:**
* `setup()` and `loop()`: initializes Serial, WiFi, LoRaWAN, queues, and FreeRTOS tasks.
* `TaskSample`: acquires and queues data.
* `TaskAnalyze`: applies filters, runs FFT to adapt the sampling rate, and aggregates 5s averages while computing TPR, FPR, and MER.
* `TaskTransmit`: publishes 5s averages via MQTT (Wi-Fi) and every 30s via LoRaWAN to respect duty cycle limits.

### Power Monitor (`src/power_monitor.cpp`)
* **Execution**: runs on the secondary ESP32 using custom I2C pins (SDA 41, SCL 42) for proper Heltec board mapping. 
* **Logic**: samples INA219 current and voltage every 10ms, calculating average power every 5s to perfectly align with the edge node's reports.


## 3. Performance Evaluation

### Part 1: Baseline vs Adaptive Sampling (Non-Bonus)
Comparison performed on **Signal 1** $2\sin(2\pi3t)+4\sin(2\pi5t)$.

* **Baseline / oversampling (Run Mode 1):** samples blindly at the defined assignment baseline frequency (100 Hz), without analyzing the informational content.
    * *Energy consumption:* ~407 mW average (consistently high).
    * *Data volume (overhead):* 502 samples processed internally every 5 seconds.
* **Adaptive sampling (Run Mode 2):** uses the FFT on a 128-sample window to identify the dominant frequency (~5.47 Hz). Applying Nyquist's theorem, it recalculates the task delay to 83 ms (~12 Hz).
    * *Energy consumption:* ~238 mW average during standard cycles (~41% reduction).
    * *Data volume (overhead):* 61 samples processed internally every 5 seconds (~88% reduction).
* **Average end-to-end latency** (common to both):
    * *Wi-Fi/MQTT:* 12 - 23 ms.
    * *LoRaWAN/TTN:* 1510 - 1550 ms.

### Part 2: Different Input Signal Types (Bonus)
Three signals with different dynamics were analyzed in adaptive mode (Run Mode 2):

* **Signal 1 (slow mixed):** $2\sin(2\pi3t)+4\sin(2\pi5t)$ | FFT: 5.47 Hz | delay: 83ms | volume: 61 samples.
* **Signal 2 (fast mixed):** $3\sin(2\pi2t)+1.5\sin(2\pi10t)$ | FFT: 10.16 Hz | delay: 45ms | volume: 112 samples.
* **Signal 3 (slow single):** $5\sin(2\pi4t)$ | FFT: 4.69 Hz | delay: 90ms | volume: 56 samples.
* **Considerations:** the adaptation works proportionally to the signal's bandwidth. Signal 3 (the slowest) allows for maximum energy and internal processing savings. Signal 2 (with a 10Hz harmonic) forces the board to wake up twice as often compared to Signal 1 to avoid aliasing, generating a volume of 112 samples.

### Part 3: Z-Score and Hampel Filters on Anomalous Signals (Bonus)
* **Impact of unfiltered anomalies (Run Mode 3):** Sudden spikes act like Dirac impulses, scattering energy across all high frequencies. The raw FFT is deceived and detects skewed f_max values (e.g., 47.66 Hz), which forces the delay back to 10ms. This completely destroys the energy savings, reverting consumption to Run 1 levels.

To solve the FFT breakdown, we implemented two anomaly-aware filters. Below is the detailed performance evaluation for all combinations of Window Size and Anomaly Probability.

#### Z-Score Filter Performance (Run Mode 4)
*Methodology: Mean & Variance. Highly efficient ($O(N)$), but susceptible to breakdown when the mean is corrupted by outliers.*

| Anomaly Prob. | Window Size | Exec Time | TPR (Avg) | FPR (Avg) | Mean Error Reduction (MER) | Energy Consumption |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1%** | **3** | ~4 µs | ~80% | ~53% | **-60%** *(Breakdown)* | ~238 mW |
| **1%** | **7** | ~4 µs | ~75% | ~7% | **+15%** | ~238 mW |
| **1%** | **15** | ~5 µs | ~85% | ~6% | **+20%** | ~238 mW |
| **5%** | **3** | ~4 µs | ~70% | ~32% | **-200%** *(Breakdown)* | ~238 mW |
| **5%** | **7** | ~4 µs | ~62% | ~8% | **+25%** | ~238 mW |
| **5%** | **15** | ~5 µs | ~58% | ~4% | **+15%** | ~238 mW |
| **10%** | **3** | ~3 µs | ~80% | ~38% | **-1000%** *(Breakdown)* | ~238 mW |
| **10%** | **7** | ~4 µs | ~80% | ~48% | **-120%** *(Breakdown)* | ~238 mW |
| **10%** | **15** | ~5 µs | ~57% | ~5% | **+15%** | ~238 mW |

**Z-Score Considerations:** 
* **Window = 3:** Fails completely, yielding massive FPRs and catastrophic negative MERs because a single outlier corrupts the mean instantly.
* **Window = 7:** Provides acceptable filtering at low anomaly rates (1% and 5%), but breaks down at 10% when multiple spikes corrupt the background mean.
* **Window = 15:** Improves statistical resilience against outliers, maintaining a positive MER even at 10% anomalies, but struggles to achieve high True Positive Rates (`~57%`) as the expanded mean absorbs and dilutes the spikes.

---

#### Hampel Filter Performance (Run Mode 5)
*Methodology: Median & MAD. Computationally heavier due to array sorting, but incredibly robust against dense outliers.*

| Anomaly Prob. | Window Size | Exec Time | TPR (Avg) | FPR (Avg) | Mean Error Reduction (MER) | Energy Consumption |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1%** | **3** | ~5 µs | ~80% | ~6% | **-25%** *(Invalid stats)* | ~235 mW |
| **1%** | **7** | ~8 µs | ~80% | ~4% | **+5%** | ~235 mW |
| **1%** | **15** | ~24 µs | ~66% | ~2% | **+15%** | ~235 mW |
| **5%** | **3** | ~5 µs | ~40% | ~6% | **-50%** *(Invalid stats)* | ~235 mW |
| **5%** | **7** | ~8 µs | ~65% | ~9% | **+20%** | ~235 mW |
| **5%** | **15** | ~23 µs | ~50% | ~3% | **+25%** | ~235 mW |
| **10%** | **3** | ~5 µs | ~35% | ~6% | **-150%** *(Invalid stats)* | ~235 mW |
| **10%** | **7** | ~8 µs | ~65% | ~7% | **+35%** | ~235 mW |
| **10%** | **15** | ~24 µs | ~35% | ~1% | **+15%** | ~235 mW |

**Hampel Considerations (Window Size Trade-off):**
* **Window = 3:** Fails statistically. 3 samples are not enough background data to properly identify a median, causing the filter to mistake regular noise for anomalies (Negative MER).
* **Window = 15:** Guarantees flawless detection and tiny FPR, but the array sorting algorithm causes CPU execution times to spike logarithmically/exponentially (`>24 µs`).
* **Window = 7:** The sweet spot. It successfully cleans the signal yielding consistent positive MERs (up to `+35%`), keeps FPR extremely low, and balances reliable median statistics with low execution time (`~8 µs` per sample).

## 4. How to Setup and Run

### Step 1: Hardware Configuration and Wiring
1. Place the two ESP32s on two separate breadboards.
2. **Battery Wiring:** Connect the positive wire (`+`) of the battery to the `VIN+` pin of the INA219, and connect the `VIN-` pin of the INA219 to the `5V` input of the **Node** ESP32. Connect the negative wire (`-`) of the battery directly to the `GND` pin of the **Node** ESP32.
3. Connect the VCC and GND power supply of the INA219 to the **Monitor** ESP32.
4. Connect the I2C pins of the INA219 to the **Monitor** ESP32: `SDA` to Pin 41, `SCL` to Pin 42.

### Step 2: Software Configuration
In the `src/main_node.cpp` file:
* Insert your The Things Network (TTN) identifiers: `joinEUI`, `devEUI` and `appKey`.
* Insert your Wi-Fi credentials into the `ssid` and `password` strings.
* (Optional) Configure the MQTT server and topic via `mqtt_server` and `mqtt_topic`.

### Step 3: Compilation and Upload (PlatformIO)
Both firmwares are managed within the project. Since both code files are located in the same `src/` folder, **you must specify the correct target environment (`env`)** in PlatformIO before performing any build or upload operation.
1. Open the `main_node.cpp` file and select a mode at the `#define RUN_MODE X` line (from 1 to 5).
2. Select the correct Environment for the Node ESP in PlatformIO and click Upload. **Important:** Disconnect the battery from the Node and disconnect the INA219 before uploading to avoid conflicts on the USB Serial port.
3. Select the Environment for the Monitor ESP, compile, and upload `power_monitor.cpp`.

### Step 4: Synchronization and Data Observation
* **Transmission Results (Node-Only Execution):** If you are only running the code on the Node without measuring power consumption, **the battery must be disconnected**. Power the Node directly via USB, connect it to the PC, and open its Serial Monitor (115200 baud). You will be able to observe the FFT processes, execution times, latencies, and error metrics in real time.
* **Consumption Results (Dual-ESP Monitoring):** To perform energy measurements, **the battery must be connected** so that power flows through the INA219 to the Node. Connect the Monitor ESP to the PC. As soon as the Node's Serial Monitor prints `>>> STARTING RUN_MODE X | PHASE 0 <<<`, **physically press the RST (Reset) button** on the Monitor ESP.
* This manual action aligns the timers perfectly. From that moment on, every 5-second aggregated printout from the Monitor will correspond to the exact power consumption in milliWatts for the same 5-second window calculated by the Node.

---

## 5. LLM Usage
The main questions I asked to LLM was about:

### HiveMQ
* How to set up a connection to an mqtt server?
* How to use HiveMQ and what I have to write as topic?

### The Things Network (TTN)
* How to set up a connection to TTN?
* How to create an end device on my console, what I have to put in the personalized fields as joinEUI devEUI appKey?
* Why the message on the "live data" section is still "DevNonce already been used"? How can I fix that?
* Why I can't read the data sent to TTN in a good format?
* Tell me the code that I have to write on the "formatter type" as "formatter code" in the "payload formatters" section in order to read data sent.

### Hardware
* What is the hardware setup i need to measure accurately the energy consumption for all the cases requested?
* How I should connect the 2 esp32 with the INA219 and a battery for the Node?

### Code Implementation
* How I should structure the code in order to analyze separately every signal and case requested?
* How to generate Gaussian noise?
* How to apply a Z-Score and Hampel filter?
* How to iterate the JOIN request to TTN?

### LLM Limitations
The LLM used (Gemini) showed some notable defects during the development of this project:
* It doesn't know well all the latest settings and updated versions for HiveMQ and TTN, making its setup instructions often imprecise.
* It lacks precise knowledge about the specific hardware board versions used in this project, sometimes struggling with the exact pinouts or default configurations.
* While generating or modifying code, it often loses sight of the overall assignment's goal, going off-track by focusing too much on a single specific request at the expense of the whole system architecture.