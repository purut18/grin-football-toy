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

// Total number of physical IR sensor modules integrated into the hardware assembly.
// Updated to 3 to support the newly added sensor row placed behind the goalkeeper.
// Technically, each sensor module acts as a sub-array of phototransistor components.
const int NUM_SENSORS = 3;

// Number of individual IR reflection photodiode/phototransistor channels per sensor module.
// The TCRT5000-based Smartelex modules contain exactly 5 discrete emitter-receiver pairs.
const int CHANNELS_PER_SENSOR = 5;

// Total quantity of analog signal channels connected to the multiplexer input lines.
// Mathematically, this is the product of the number of sensor modules (3) and the channels per module (5).
// 3 sensors * 5 channels = 15 total channels multiplexed via time-division into a single analog pin.
const int TOTAL_CHANNELS = NUM_SENSORS * CHANNELS_PER_SENSOR;

// Digital pin assignments for the MUX address lines (S0-S3).
// We use board-specific pin macros (D2, D3, D6, D7) rather than raw register numbers or hardware GPIO IDs
// to comply with the board layout and to avoid resource conflicts with the FDCAN1 CAN Bus peripheral 
// mapped by default to D4 and D5 in the Zephyr RTOS device tree configuration.
const int MUX_S0_PIN = D2;  // Address bit 0 (least significant bit, LSB) for 4-bit binary channel selection
const int MUX_S1_PIN = D3;  // Address bit 1 for 4-bit binary channel selection
const int MUX_S2_PIN = D6;  // Address bit 2 (shifted from D4 to prevent FDCAN1 transmit pin conflicts)
const int MUX_S3_PIN = D7;  // Address bit 3 (most significant bit, MSB, shifted from D5 to prevent FDCAN1 receive pin conflicts)

// Analog pin configured to receive the common multiplexed signal output (SIG) of the CD74HC4067.
// The selected multiplexer channel's voltage level is electrically connected to A0 for ADC sampling.
const int MUX_SIG_PIN = A0;

// MUX channel-to-sensor routing mapping table.
// Maps each logical sensor index and channel index to a physical multiplexer input port (C0 to C14).
// C0-C4 route Sensor 1 (front-most row).
// C5-C9 route Sensor 2 (mid row).
// C10-C14 route Sensor 3 (rear-most row, situated behind the goalkeeper for goal detection).
const int MUX_CHANNEL_MAP[NUM_SENSORS][CHANNELS_PER_SENSOR] = {
  {0, 1, 2, 3, 4},     // Sensor 1: MUX channels C0–C4
  {5, 6, 7, 8, 9},     // Sensor 2: MUX channels C5–C9
  {10, 11, 12, 13, 14} // Sensor 3: MUX channels C10–C14
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

// Instantiate the global multiplexer hardware abstraction layer.
// Passed with the digital control pins (S0-S3) and the common analog signal line (SIG) on A0.
// This instance manages the low-level digital write operations and transient settling delays.
CD74HC4067Mux mux(MUX_S0_PIN, MUX_S1_PIN, MUX_S2_PIN, MUX_S3_PIN, MUX_SIG_PIN);

// Instantiate MuxAnalogTracker wrappers for each of the three physical sensor rows.
// Each tracker is bound to the shared multiplexer controller and receives its respective channel configuration
// from the MUX_CHANNEL_MAP. This isolates the ADC read logic from the signal processing math.
MuxAnalogTracker tracker1(&mux, MUX_CHANNEL_MAP[0]); // Sensor Row 1 (Entry row)
MuxAnalogTracker tracker2(&mux, MUX_CHANNEL_MAP[1]); // Sensor Row 2 (Middle row)
MuxAnalogTracker tracker3(&mux, MUX_CHANNEL_MAP[2]); // Sensor Row 3 (Goal row behind keeper)

// Declare the array of tracker pointers to allow clean loop-based iteration during sampling.
// Dimensioned dynamically by NUM_SENSORS (3) to prevent buffer overflows or compilation bounds errors.
MuxAnalogTracker* trackers[NUM_SENSORS] = {&tracker1, &tracker2, &tracker3};

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

// ==============================================================================
// STATE MACHINE CONFIGURATION
// ==============================================================================

// Enumeration representing the operational states of the ball-tracking state machine.
// STATE_WAITING_FOR_SHOT: The system is idle, scanning for a rising edge trigger on Sensor 1 or Sensor 2.
// STATE_WAITING_FOR_GOAL: A shot is in progress. The system accumulates triggers and watches Sensor 3 (goal line).
// STATE_COOLDOWN: A shot has finalized. The system ignores inputs for 2 seconds to allow the field to clear.
enum SystemState {
  STATE_WAITING_FOR_SHOT,
  STATE_WAITING_FOR_GOAL,
  STATE_COOLDOWN
};

// Global control variables for orchestrating state transitions and event aggregation windows.
SystemState currentState = STATE_WAITING_FOR_SHOT;
unsigned long shotStartTime = 0;      // System millisecond timestamp when the shot was initiated.
unsigned long goalTriggerTime = 0;    // System millisecond timestamp when the goal line (Sensor 3) first triggered.
bool goalDetected = false;            // Latched boolean flag denoting whether Sensor 3 was activated during the shot.
unsigned long cooldownStartTime = 0;  // System millisecond timestamp when the post-shot cooldown period commenced.

// Timestamp tracker (in milliseconds) for when each of the NUM_SENSORS rows is first activated.
// Used to compute relative trigger timing (T+X ms) relative to the initial shot trigger.
// This is required to capture the speed/timing dynamics of the ball's movement across the rows.
unsigned long sensorRowFirstTriggerTime[NUM_SENSORS];

// Timeouts and settling windows (in milliseconds) used to govern state transition logic.
const unsigned long SHOT_TIMEOUT_MS = 1500;  // Maximum duration to wait for a goal trigger before declaring a miss/save.
const unsigned long GOAL_SETTLE_MS = 80;     // Settling period after goal trigger to capture the full crossing profile.
const unsigned long COOLDOWN_MS = 2000;      // Hard post-shot system lockout duration (2.0 seconds) to reset and wait for next play.

// ==============================================================================
// INITIALIZATION HELPER FUNCTIONS
// ==============================================================================

// initializeStateArrays — Resets all state tracking variables, filters, and baselines to clean initial values.
// Called during setup() to bootstrap the system, and can be invoked dynamically for hot-resets.
void initializeStateArrays() {
  currentState = STATE_WAITING_FOR_SHOT;
  shotStartTime = 0;
  goalTriggerTime = 0;
  goalDetected = false;
  cooldownStartTime = 0;
  
  // Initialize the first trigger timestamp array for each sensor row to 0 (untriggered).
  // This resets timing tracking for all sensors before a new shot event begins.
  for (int s = 0; s < NUM_SENSORS; s++) {
    sensorRowFirstTriggerTime[s] = 0;
  }
  
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

  // Output diagnostic details to Serial console for operational verification of the hardware setup.
  // These lines detail the multiplexer and pin routing mapping for operators troubleshooting connection issues.
  Serial.println("=====================================================");
  Serial.println("  PRODUCTION TRIPLE IR TRACKER via CD74HC4067 MUX");
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
  Serial.println("Sensor 3 -> MUX: IR1=C10 IR2=C11 IR3=C12 IR4=C13 IR5=C14");
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

  // ============================================================================
  // PHASE 2: SHOT DETECTION & GOAL EVALUATION STATE MACHINE
  // ============================================================================
  // The system uses a state machine to track the ball's transition across the pitch:
  // 1. STATE_WAITING_FOR_SHOT: Waits for a rising-edge trigger on Sensor 1 and/or 2.
  // 2. STATE_WAITING_FOR_GOAL: Shot is active. Wait for Sensor 3 trigger (goal) or timeout.
  // 3. STATE_COOLDOWN: Lock system for 2 seconds to allow ball clear, maintaining baseline calibration.
  switch (currentState) {
    
    case STATE_WAITING_FOR_SHOT: {
      bool shotDetected = false;
      // Loop through the front two sensors (Sensor 1 and Sensor 2) to detect ball entry.
      for (int s = 0; s < 2; s++) {
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          // Detect shot starting on a clean rising edge (untriggered to triggered state transition)
          if (currentTriggerState[s][c] && !prevTriggerState[s][c]) {
            sensorsTriggeredInEvent[s][c] = true;
            shotDetected = true;
          }
        }
      }
      
      // If a rising edge occurred on Sensor 1 or Sensor 2, start the shot tracking window
      if (shotDetected) {
        currentState = STATE_WAITING_FOR_GOAL;
        shotStartTime = millis();
        goalDetected = false;
        
        // Set the first trigger timestamp (T+0ms) for the row(s) that initiated the shot.
        // This marks the baseline time reference (T=0) for relative speed tracking.
        for (int s = 0; s < 2; s++) {
          for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
            if (sensorsTriggeredInEvent[s][c]) {
              sensorRowFirstTriggerTime[s] = shotStartTime;
              break; // Set once per row
            }
          }
        }
        
        Serial.println(">>> Shot detected! Waiting for goal evaluation...");
      }
      break;
    }
    
    case STATE_WAITING_FOR_GOAL: {
      // Accumulate all active channel triggers on Sensor 1, Sensor 2, and Sensor 3 during shot window.
      // We check absolute currentTriggerState so that triggers at any point during the window are latched.
      for (int s = 0; s < NUM_SENSORS; s++) {
        bool rowTriggeredThisIteration = false;
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          if (currentTriggerState[s][c]) {
            sensorsTriggeredInEvent[s][c] = true;
            rowTriggeredThisIteration = true;
          }
        }
        
        // If this sensor row registered an active trigger during this iteration and its 
        // first trigger timestamp is still unset (0), record the current system time.
        // This captures the exact time the ball arrived at this physical row.
        if (rowTriggeredThisIteration && sensorRowFirstTriggerTime[s] == 0) {
          sensorRowFirstTriggerTime[s] = millis();
        }
      }
      
      // Look for the goal line (Sensor 3) first trigger event if not already detected
      if (!goalDetected) {
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          if (currentTriggerState[2][c]) {
            goalDetected = true;
            // Align goalTriggerTime with the first trigger time of Sensor 3 recorded above.
            goalTriggerTime = sensorRowFirstTriggerTime[2];
            Serial.println(">>> Goal sensor activated! Settling...");
            break;
          }
        }
      }
      
      // Check if we should finalize the current shot event and output results:
      // - If goal line (Sensor 3) triggered: wait GOAL_SETTLE_MS (80ms) to capture full transit width.
      // - If goal line did not trigger: wait for SHOT_TIMEOUT_MS (1500ms) to elapse.
      bool finalizeShot = false;
      if (goalDetected && (millis() - goalTriggerTime >= GOAL_SETTLE_MS)) {
        finalizeShot = true;
      } else if (millis() - shotStartTime >= SHOT_TIMEOUT_MS) {
        finalizeShot = true;
      }
      
      if (finalizeShot) {
        // Output consolidated event state with relative trigger timestamps for each sensor row.
        // Format matches: Sx (T+XXms): [c0,c1,c2,c3,c4] or Sx (N/A): [0,0,0,0,0] if not crossed.
        for (int s = 0; s < NUM_SENSORS; s++) {
          Serial.print("S");
          Serial.print(s + 1);
          if (sensorRowFirstTriggerTime[s] > 0) {
            // Compute relative trigger offset in milliseconds
            unsigned long relTime = sensorRowFirstTriggerTime[s] - shotStartTime;
            Serial.print(" (T+");
            Serial.print(relTime);
            Serial.print("ms): [");
          } else {
            Serial.print(" (N/A): [");
          }
          
          for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
            Serial.print(sensorsTriggeredInEvent[s][c] ? "1" : "0");
            if (c < CHANNELS_PER_SENSOR - 1) {
              Serial.print(",");
            }
          }
          Serial.println("]");
        }
        
        // Enter cooldown period to block subsequent triggers and let ball roll clear of sensors
        currentState = STATE_COOLDOWN;
        cooldownStartTime = millis();
        Serial.println(">>> Shot finalized. Entering 2-second cooldown...");
      }
      break;
    }
    
    case STATE_COOLDOWN: {
      // During cooldown, do not start new shots or aggregate events.
      // Once the cooldown period (2.0s) has elapsed, clear event triggers and return to idle.
      if (millis() - cooldownStartTime >= COOLDOWN_MS) {
        // Reset the event triggers array and timestamps to prepare for next shot
        for (int s = 0; s < NUM_SENSORS; s++) {
          sensorRowFirstTriggerTime[s] = 0;
          for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
            sensorsTriggeredInEvent[s][c] = false;
          }
        }
        currentState = STATE_WAITING_FOR_SHOT;
        Serial.println(">>> Cooldown complete. System ready for next shot.");
      }
      break;
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
