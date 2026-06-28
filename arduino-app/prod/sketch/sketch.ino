/*
 * sketch.ino — Production Dual IR Sensor Tracker via CD74HC4067 MUX (Safe Pinout)
 * ==============================================================================
 *
 * This production sketch runs on the STM32U585 microcontroller of the Arduino
 * Uno Q. It interfaces with TWO Smartelex 5-Channel Tracker Sensors
 * (TCRT5000-based reflection sensors) through a single SmartElex CD74HC4067
 * 16-channel analog/digital multiplexer (MUX) breakout board.
 *
 * Instead of reading each sensor's 5 analog outputs directly from 5 dedicated
 * Arduino analog pins (as in the IR-sensor-test), all 10 sensor lines
 * (5 per sensor × 2 sensors = 10 channels) are routed through the MUX's
 * 16 input channels (C0–C15), and a single Arduino analog pin (SIG) reads
 * the selected channel's voltage.
 *
 * The MUX channel is selected by writing a 4-bit binary address to digital
 * pins S0–S3 on the Arduino. The CD74HC4067 internally connects the addressed
 * channel input to the common SIG output line.
 *
 * Technical Specifications & Design Concepts:
 * -------------------------------------------
 * 1. SmartElex CD74HC4067 16-Channel MUX Breakout:
 *    - IC: Texas Instruments CD74HC4067 (CMOS analog multiplexer/demultiplexer).
 *    - Control Interface: 4 digital address lines (S0, S1, S2, S3) form a
 *      4-bit binary selector.
 *    - EN (Enable): Active-LOW. Must be tied to GND to enable the chip.
 *    - Signal Path: Bidirectional. The selected Cx channel is electrically
 *      connected to SIG with very low on-resistance (~70Ω typ at 3.3V).
 *
 * 2. safe GPIO Pin Mapping (Avoiding CAN Bus Collision):
 *    - D4 and D5 are multiplexed with CAN Bus (FDCAN1) under Zephyr RTOS and are
 *      active by default. Driving them as outputs fails, causing them to float.
 *    - Solution: We map MUX S2 and S3 to safe GPIO pins D6 and D7.
 *
 * 3. MUX Channel Mapping:
 *    - Sensor 1 (5 channels): C0=IR1_S1, C1=IR2_S1, C2=IR3_S1, C3=IR4_S1, C4=IR5_S1
 *    - Sensor 2 (5 channels): C5=IR1_S2, C6=IR2_S2, C7=IR3_S2, C8=IR4_S2, C9=IR5_S2
 *
 * 4. Wiring Interface (MUX → Arduino Uno Q):
 *    - VCC  → 3.3V Pin (powers the MUX IC and both sensor modules)
 *    - GND  → GND Pin (common ground reference)
 *    - EN   → GND Pin (active-LOW enable; permanently enabled)
 *    - S0   → Digital Pin D2 (address bit 0, LSB)
 *    - S1   → Digital Pin D3 (address bit 1)
 *    - S2   → Digital Pin D6 (address bit 2) [Moved from D4]
 *    - S3   → Digital Pin D7 (address bit 3, MSB) [Moved from D5]
 *    - SIG  → Analog Pin A0 (common analog signal output from MUX)
 */

#include <Arduino_RouterBridge.h>

// ==============================================================================
// HARDWARE CONFIGURATION CONSTANTS
// ==============================================================================

// Total number of IR sensor modules connected to the system.
const int NUM_SENSORS = 2;

// Number of individual IR channels per sensor module.
const int CHANNELS_PER_SENSOR = 5;

// Total number of analog channels routed through the MUX.
const int TOTAL_CHANNELS = NUM_SENSORS * CHANNELS_PER_SENSOR;

// Digital pin assignments for the MUX address lines.
// We use the board pin macros D2, D3, D6, D7 to avoid CAN Bus conflict on D4/D5.
const int MUX_S0_PIN = D2;  // Address bit 0 (least significant bit)
const int MUX_S1_PIN = D3;  // Address bit 1
const int MUX_S2_PIN = D6;  // Address bit 2 [Moved from D4]
const int MUX_S3_PIN = D7;  // Address bit 3 (most significant bit) [Moved from D5]

// Analog pin connected to the MUX's common SIG (signal) output.
const int MUX_SIG_PIN = A0;

// MUX channel-to-sensor mapping table.
const int MUX_CHANNEL_MAP[NUM_SENSORS][CHANNELS_PER_SENSOR] = {
  {0, 1, 2, 3, 4},   // Sensor 1: MUX channels C0–C4
  {5, 6, 7, 8, 9}    // Sensor 2: MUX channels C5–C9
};

// ==============================================================================
// CD74HC4067Mux CLASS
// ==============================================================================

class CD74HC4067Mux {
private:
  int s0Pin, s1Pin, s2Pin, s3Pin;
  int sigPin;

public:
  CD74HC4067Mux(int s0, int s1, int s2, int s3, int sig) {
    s0Pin = s0;
    s1Pin = s1;
    s2Pin = s2;
    s3Pin = s3;
    sigPin = sig;
  }

  void begin() {
    pinMode(s0Pin, OUTPUT);
    pinMode(s1Pin, OUTPUT);
    pinMode(s2Pin, OUTPUT);
    pinMode(s3Pin, OUTPUT);
    
    // Note: We do NOT call pinMode(sigPin, INPUT) on A0 to prevent any potential
    // pinmux assert conflicts with other peripherals under Zephyr.
  }

  void selectChannel(int channel) {
    digitalWrite(s0Pin, channel & 1);
    digitalWrite(s1Pin, (channel >> 1) & 1);
    digitalWrite(s2Pin, (channel >> 2) & 1);
    digitalWrite(s3Pin, (channel >> 3) & 1);
  }

  int readChannel(int channel) {
    if (channel < 0 || channel > 15) {
      return 0;
    }

    selectChannel(channel);
    
    // Settle time for external RC line capacitance: 50 microseconds
    delayMicroseconds(50);

    // Oversample 4 times and average to eliminate bus RPC transient noise.
    long sum = 0;
    for (int i = 0; i < 4; i++) {
      sum += analogRead(sigPin);
      delayMicroseconds(5);
    }

    return (int)(sum / 4);
  }
};

// ==============================================================================
// MuxAnalogTracker CLASS
// ==============================================================================

class MuxAnalogTracker {
private:
  CD74HC4067Mux* mux;
  int muxChannels[CHANNELS_PER_SENSOR];
  int adcMaxVal;

public:
  MuxAnalogTracker(CD74HC4067Mux* muxPtr, const int channels[CHANNELS_PER_SENSOR]) {
    mux = muxPtr;
    for (int i = 0; i < CHANNELS_PER_SENSOR; i++) {
      muxChannels[i] = channels[i];
    }
    adcMaxVal = 1023;
  }

  int readRaw(int index) {
    if (index < 0 || index >= CHANNELS_PER_SENSOR) {
      return 0;
    }
    return mux->readChannel(muxChannels[index]);
  }

  float getPercentageFromRaw(int rawVal) {
    float fraction = (float)rawVal / adcMaxVal;
    float percentage = (1.0 - fraction) * 100.0;
    return percentage;
  }

  float getPercentage(int index) {
    if (index < 0 || index >= CHANNELS_PER_SENSOR) {
      return 0.0;
    }
    int rawVal = readRaw(index);
    return getPercentageFromRaw(rawVal);
  }
};

// ==============================================================================
// GLOBAL INSTANCES
// ==============================================================================

CD74HC4067Mux mux(MUX_S0_PIN, MUX_S1_PIN, MUX_S2_PIN, MUX_S3_PIN, MUX_SIG_PIN);

MuxAnalogTracker tracker1(&mux, MUX_CHANNEL_MAP[0]);
MuxAnalogTracker tracker2(&mux, MUX_CHANNEL_MAP[1]);

MuxAnalogTracker* trackers[NUM_SENSORS] = {&tracker1, &tracker2};

// ==============================================================================
// ADAPTIVE THRESHOLDING & SIGNAL PROCESSING STATE ARRAYS
// ==============================================================================

const int MOVING_AVERAGE_WINDOW_SIZE = 1;

int rawReadingsHistory[NUM_SENSORS][CHANNELS_PER_SENSOR][MOVING_AVERAGE_WINDOW_SIZE];
int historyWriteIndex = 0;

int calibrationBaseline[NUM_SENSORS][CHANNELS_PER_SENSOR];
float calibrationBaselineFloat[NUM_SENSORS][CHANNELS_PER_SENSOR];

bool prevTriggerState[NUM_SENSORS][CHANNELS_PER_SENSOR];
bool isRecovering[NUM_SENSORS][CHANNELS_PER_SENSOR];

unsigned long triggerStartTime[NUM_SENSORS][CHANNELS_PER_SENSOR];
int minRawDuringTrigger[NUM_SENSORS][CHANNELS_PER_SENSOR];

float filteredReadings[NUM_SENSORS][CHANNELS_PER_SENSOR];
float prevFilteredReadings[NUM_SENSORS][CHANNELS_PER_SENSOR];

float noiseVarianceFloat[NUM_SENSORS][CHANNELS_PER_SENSOR];

int dynamicTriggerThresholds[NUM_SENSORS][CHANNELS_PER_SENSOR];
int dynamicReleaseThresholds[NUM_SENSORS][CHANNELS_PER_SENSOR];

bool sensorsTriggeredInEvent[NUM_SENSORS][CHANNELS_PER_SENSOR];

unsigned long lastEventPrintTime = 0;
const unsigned long EVENT_COOLDOWN_MS = 500;

// Event accumulation state. When a trigger is first detected, we accumulate
// triggers for a small window (150ms) to allow the ball to roll across both
// sensor rows before printing a combined event vector.
bool isAccumulatingEvent = false;
unsigned long eventAccumulationStartTime = 0;
const unsigned long EVENT_ACCUMULATION_MS = 150;

// ==============================================================================
// INITIALIZATION HELPER FUNCTIONS
// ==============================================================================

void initializeStateArrays() {
  isAccumulatingEvent = false;
  eventAccumulationStartTime = 0;
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      calibrationBaseline[s][c] = 0;
      calibrationBaselineFloat[s][c] = 0.0;
      prevTriggerState[s][c] = false;
      isRecovering[s][c] = false;
      triggerStartTime[s][c] = 0;
      minRawDuringTrigger[s][c] = 1023;
      filteredReadings[s][c] = 0.0;
      prevFilteredReadings[s][c] = 0.0;
      noiseVarianceFloat[s][c] = 5.0;
      dynamicTriggerThresholds[s][c] = 30;
      dynamicReleaseThresholds[s][c] = 20;
      sensorsTriggeredInEvent[s][c] = false;
    }
  }
}

// ==============================================================================
// SETUP
// ==============================================================================

void setup() {
  Serial.begin(9600);
  Bridge.begin();

  mux.begin();
  initializeStateArrays();

  Serial.println("Stabilizing hardware components... Keep pitch clear!");
  delay(2000);

  // Seed history
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      int initialVal = trackers[s]->readRaw(c);
      for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
        rawReadingsHistory[s][c][sample] = initialVal;
      }
    }
  }

  // Settle history
  for (int sample = 0; sample < 50; sample++) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        rawReadingsHistory[s][c][historyWriteIndex] = trackers[s]->readRaw(c);
      }
    }
    historyWriteIndex = (historyWriteIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;
    delay(2);
  }

  // Calculate baselines
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      long sum = 0;
      for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
        sum += rawReadingsHistory[s][c][sample];
      }
      float average = (float)sum / MOVING_AVERAGE_WINDOW_SIZE;
      calibrationBaselineFloat[s][c] = average;
      calibrationBaseline[s][c] = (int)average;
    }
  }

  // Print baselines
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.print("Sensor ");
    Serial.print(s + 1);
    Serial.print(" Baselines: ");
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      Serial.print("IR");
      Serial.print(c + 1);
      Serial.print(":");
      Serial.print(calibrationBaseline[s][c]);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.println("=====================================================");
  Serial.println("  PRODUCTION DUAL IR TRACKER via CD74HC4067 MUX");
  Serial.println("=====================================================");
  Serial.println("MUX Wiring (CD74HC4067 -> Arduino UNO Q):");
  Serial.println("  VCC (Power)    -> 3.3V Pin");
  Serial.println("  GND (Ground)   -> GND Pin");
  Serial.println("  EN  (Enable)   -> GND Pin (active-LOW, always ON)");
  Serial.println("  S0  (Addr b0)  -> Digital Pin D2");
  Serial.println("  S1  (Addr b1)  -> Digital Pin D3");
  Serial.println("  S2  (Addr b2)  -> Digital Pin D6 (was D4)");
  Serial.println("  S3  (Addr b3)  -> Digital Pin D7 (was D5)");
  Serial.println("  SIG (Signal)   -> Analog Pin A0");
  Serial.println("-----------------------------------------------------");
  Serial.println("Sensor 1 -> MUX: IR1=C0 IR2=C1 IR3=C2 IR4=C3 IR5=C4");
  Serial.println("Sensor 2 -> MUX: IR1=C5 IR2=C6 IR3=C7 IR4=C8 IR5=C9");
  Serial.println("=====================================================");
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================

void loop() {
  static unsigned long lastDebugPrint = 0;

  int currentRawReadings[NUM_SENSORS][CHANNELS_PER_SENSOR];
  bool currentTriggerState[NUM_SENSORS][CHANNELS_PER_SENSOR];

  // PHASE 1: READ, FILTER, AND EVALUATE
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      currentRawReadings[s][c] = trackers[s]->readRaw(c);

      rawReadingsHistory[s][c][historyWriteIndex] = currentRawReadings[s][c];

      long sum = 0;
      for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
        sum += rawReadingsHistory[s][c][sample];
      }
      filteredReadings[s][c] = (float)sum / MOVING_AVERAGE_WINDOW_SIZE;

      int diff = calibrationBaseline[s][c] - (int)filteredReadings[s][c];

      bool newState = prevTriggerState[s][c];
      if (!prevTriggerState[s][c]) {
        if (diff >= dynamicTriggerThresholds[s][c]) newState = true;
      } else {
        if (diff < dynamicReleaseThresholds[s][c]) newState = false;
      }
      currentTriggerState[s][c] = newState;

      if (diff > 0 && !currentTriggerState[s][c]) {
        noiseVarianceFloat[s][c] = (noiseVarianceFloat[s][c] * 0.99) + ((float)diff * 0.01);
      }

      int dynamicTrigger = max(30, (int)(noiseVarianceFloat[s][c] * 3.0));
      int dynamicRelease = max(20, (int)(noiseVarianceFloat[s][c] * 1.5));
      dynamicTriggerThresholds[s][c] = dynamicTrigger;
      dynamicReleaseThresholds[s][c] = dynamicRelease;

      int upwardDiff = filteredReadings[s][c] - calibrationBaselineFloat[s][c];

      if (upwardDiff > dynamicTriggerThresholds[s][c]) {
        isRecovering[s][c] = true;
      }
      if (upwardDiff <= 0) {
        isRecovering[s][c] = false;
      }

      if (isRecovering[s][c]) {
        calibrationBaselineFloat[s][c] = (calibrationBaselineFloat[s][c] * 0.9) + (filteredReadings[s][c] * 0.1);
      } else {
        if (currentTriggerState[s][c] && (millis() - triggerStartTime[s][c] > 500)) {
          calibrationBaselineFloat[s][c] = (calibrationBaselineFloat[s][c] * 0.9) + (filteredReadings[s][c] * 0.1);
        } else {
          calibrationBaselineFloat[s][c] = (calibrationBaselineFloat[s][c] * 0.999) + (filteredReadings[s][c] * 0.001);
        }
      }

      calibrationBaseline[s][c] = (int)calibrationBaselineFloat[s][c];
    }
  }

  // PHASE 2: EVENT ACCUMULATION & AGGREGATION (ACROSS DUAL SENSOR ROWS)
  // To correctly capture ball transits that span both physical sensor rows,
  // we do not print immediately. Instead, when a trigger first occurs, we start
  // a 150ms accumulation window. All triggers from both sensors during this window
  // are consolidated into a single combined output vector.
  if (millis() - lastEventPrintTime > EVENT_COOLDOWN_MS) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        if (currentTriggerState[s][c]) {
          // If a new rising edge is detected, begin/extend trigger logging
          if (!prevTriggerState[s][c]) {
            triggerStartTime[s][c] = millis();
            if (!isAccumulatingEvent) {
              isAccumulatingEvent = true;
              eventAccumulationStartTime = millis();
            }
          }

          // If we are currently accumulating an event, capture this channel trigger
          if (isAccumulatingEvent) {
            sensorsTriggeredInEvent[s][c] = true;
          }
        }
      }
    }

    // Check if the accumulation window has closed (150ms elapsed since the first trigger)
    if (isAccumulatingEvent && (millis() - eventAccumulationStartTime >= EVENT_ACCUMULATION_MS)) {
      Serial.print("[[");
      // Output consolidated Sensor 1 triggers
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        Serial.print(sensorsTriggeredInEvent[0][c] ? "1" : "0");
        if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
        sensorsTriggeredInEvent[0][c] = false; // Reset for next event
      }
      Serial.print("],[");
      // Output consolidated Sensor 2 triggers
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        Serial.print(sensorsTriggeredInEvent[1][c] ? "1" : "0");
        if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
        sensorsTriggeredInEvent[1][c] = false; // Reset for next event
      }
      Serial.println("]]");

      // Reset accumulation flag and engage the 500ms post-event cooldown
      isAccumulatingEvent = false;
      lastEventPrintTime = millis();
    }
  } else {
    // During post-event cooldown, still record trigger timestamps for adaptive baseline calculation.
    for (int s = 0; s < NUM_SENSORS; s++) {
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        if (currentTriggerState[s][c] && !prevTriggerState[s][c]) {
          triggerStartTime[s][c] = millis();
        }
      }
    }
  }

  // PHASE 3: DIAGNOSTIC HEARTBEAT
  if (millis() - lastDebugPrint > 1000) {
    lastDebugPrint = millis();
    for (int s = 0; s < NUM_SENSORS; s++) {
      Serial.print("HB S");
      Serial.print(s + 1);
      Serial.print(" | BL:[");
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        Serial.print(calibrationBaseline[s][c]);
        if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
      }
      Serial.print("] | Raw:[");
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        Serial.print(currentRawReadings[s][c]);
        if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
      }
      Serial.print("] | DT:[");
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        Serial.print(dynamicTriggerThresholds[s][c]);
        if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
      }
      Serial.println("]");
    }
  }

  // PHASE 4: STATE BUFFER UPDATE
  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
      prevTriggerState[s][c] = currentTriggerState[s][c];
      prevFilteredReadings[s][c] = filteredReadings[s][c];
    }
  }

  historyWriteIndex = (historyWriteIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;
  delay(2);
}
