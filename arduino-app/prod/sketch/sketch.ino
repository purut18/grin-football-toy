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
#include "Servo.h"

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
// S1 has been restored to D3 to match the user's physical multiplexer wiring.
// S2 and S3 are mapped to D6 and D7 to avoid CAN Bus conflicts on D4 and D5.
const int MUX_S0_PIN = D2;  // Address bit 0 (least significant bit, LSB) for 4-bit binary channel selection
const int MUX_S1_PIN = D3;  // Address bit 1 (restored to D3 for original multiplexer address wiring)
const int MUX_S2_PIN = D6;  // Address bit 2 (restored to D6)
const int MUX_S3_PIN = D7;  // Address bit 3 (restored to D7)

// Goalkeeper Servo configuration constants.
// Connect the servo signal line to D8. We use D8 because the Zephyr Servo library 
// implements software-driven timer interrupts, allowing any digital pin to be used for servo PWM.
const int SERVO_PIN = D8;
const int SERVO_MIN_ANGLE = 0;    // Servo angle in degrees for position 0 (far left column)
const int SERVO_MAX_ANGLE = 180;  // Servo angle in degrees for position 4 (far right column)
const int KEEPER_POSITIONS = 5;   // Total discrete goalkeeper columns matching the pitch width
const bool SERVO_INVERTED = true; // Set to true if physical servo direction is opposite (moves right when commanded left)

// Analog pin configured to receive the common multiplexed signal output (SIG) of the CD74HC4067.
// The selected multiplexer channel's voltage level is electrically connected to A0 for ADC sampling.
const int MUX_SIG_PIN = A0;

// MUX channel-to-sensor routing mapping table.
// Maps each logical sensor index and channel index to a physical multiplexer input port (C0 to C14).
// C0-C4 route Sensor 1 (front entry row).
// C5-C9 route Sensor 2 (middle approach row).
// C10-C14 route Sensor 3 (goal line row, situated behind the goalkeeper for goal detection).
const int MUX_CHANNEL_MAP[NUM_SENSORS][CHANNELS_PER_SENSOR] = {
  {0, 1, 2, 3, 4},     // Sensor 1 (Front Row): MUX channels C0–C4
  {5, 6, 7, 8, 9},     // Sensor 2 (Middle Row): MUX channels C5–C9
  {10, 11, 12, 13, 14} // Sensor 3 (Goal Row): MUX channels C10–C14
};

// Declare the global Servo object to control the goalkeeper's movement.
Servo goalkeeper;
int currentKeeperPos = 2; // Tracks the current active column (0-4) of the goalkeeper.
unsigned long lastServoActionTime = 0; // Timestamp of the last servo write to track motor noise transients
const unsigned long SERVO_NOISE_WINDOW_MS = 150; // Settle window (in ms) for electrical/inductive transients

// setGoalkeeperPosition — Moves the goalkeeper to one of the 5 discrete column positions (0-4).
// It clamps the input to safe boundaries, updates the tracking variable, maps the column index 
// to a corresponding servo angle, and commands the physical servo.
void setGoalkeeperPosition(int position) {
  // Enforce range clamping as a safety measure to prevent servo over-travel or mechanical binding.
  position = constrain(position, 0, KEEPER_POSITIONS - 1);
  currentKeeperPos = position;
  
  int angle;
  if (SERVO_INVERTED) {
    // Inverted mapping: position 0 -> Max Angle, position 4 -> Min Angle
    angle = map(position, 0, KEEPER_POSITIONS - 1, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);
  } else {
    // Standard mapping: position 0 -> Min Angle, position 4 -> Max Angle
    angle = map(position, 0, KEEPER_POSITIONS - 1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  }
  
  // Write the command to the servo control register.
  goalkeeper.write(angle);
  
  // Record timestamp to temporarily lock out noise baseline updates and apply noise margins
  lastServoActionTime = millis();
}

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
bool hasPredictedThisShot = false;    // State flag to ensure model prediction is requested exactly once per shot event.

// Timestamp tracker (in milliseconds) for when each of the NUM_SENSORS rows is first activated.
// Used to compute relative trigger timing (T+X ms) relative to the initial shot trigger.
// This is required to capture the speed/timing dynamics of the ball's movement across the rows.
unsigned long sensorRowFirstTriggerTime[NUM_SENSORS];

// Timeouts and settling windows (in milliseconds) used to govern state transition logic.
const unsigned long SHOT_TIMEOUT_MS = 1500;  // Maximum duration to wait for a goal trigger before declaring a miss/save.
const unsigned long GOAL_SETTLE_MS = 80;     // Settling period after goal trigger to capture the full crossing profile.
const unsigned long COOLDOWN_MS = 2000;      // Hard post-shot system lockout duration (2.0 seconds) to reset and wait for next play.

// Local policy lookup table mapping (s1_mask, s2_mask) -> optimal goalkeeper position (0 to 4).
// Dimensioned 32x32 to accommodate all possible 5-bit sensor states (0-31 for s1 and s2).
// Consumes 1024 bytes of SRAM. Entries initialized to 255 (uninitialized fallback).
uint8_t mcu_policy_table[32][32];

// updatePolicyState — RPC callback to sync model weights from MPU to MCU SRAM dynamically.
// This function is registered as a safe callback and executed on the main loop context, 
// ensuring thread-safe atomic updates to the local array.
void updatePolicyState(int s1_mask, int s2_mask, int action) {
  // Constrain inputs to prevent out-of-bounds array indexing vulnerability
  s1_mask = constrain(s1_mask, 0, 31);
  s2_mask = constrain(s2_mask, 0, 31);
  action = constrain(action, 0, KEEPER_POSITIONS - 1);
  
  mcu_policy_table[s1_mask][s2_mask] = (uint8_t)action;
}

// initializeStateArrays — Resets all state tracking variables, filters, and baselines to clean initial values.
// Called during setup() to bootstrap the system, and can be invoked dynamically for hot-resets.
void initializeStateArrays() {
  currentState = STATE_WAITING_FOR_SHOT;
  shotStartTime = 0;
  goalTriggerTime = 0;
  goalDetected = false;
  cooldownStartTime = 0;
  hasPredictedThisShot = false;

  // Initialize the local policy lookup table to 255 (meaning uninitialized fallback).
  // This ensures that any state without a learned policy yet falls back to the physics prior.
  for (int s1 = 0; s1 < 32; s1++) {
    for (int s2 = 0; s2 < 32; s2++) {
      mcu_policy_table[s1][s2] = 255;
    }
  }
  
  // Set the goalkeeper to the neutral center position at initialization.
  // This provides equal coverage of all columns at startup and resets.
  setGoalkeeperPosition(2);
  
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
  
  // Attach the goalkeeper servo pin before initializing the state arrays (which sets it to center).
  // This configures the timer-driven PWM signal generation on D3.
  goalkeeper.attach(SERVO_PIN);
  
  initializeStateArrays();

  // Register the policy update RPC callback.
  // The MPU will call this to synchronize model weights into the MCU's local SRAM lookup table.
  Bridge.provide_safe("update_policy_state", updatePolicyState);

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
  Serial.println("  S1  (Addr b1)  -> Digital Pin D3 (original wiring)");
  Serial.println("  S2  (Addr b2)  -> Digital Pin D6 (was D4)");
  Serial.println("  S3  (Addr b3)  -> Digital Pin D7 (was D5)");
  Serial.println("  SIG (Signal)   -> Analog Pin A0");
  Serial.println("  Servo (Signal) -> Digital Pin D8 (software PWM)");
  Serial.println("-----------------------------------------------------");
  Serial.println("Sensor 1 -> MUX: IR1=C0 IR2=C1 IR3=C2 IR4=C3 IR5=C4");
  Serial.println("Sensor 2 -> MUX: IR1=C5 IR2=C6 IR3=C7 IR4=C8 IR5=C9 (Middle)");
  Serial.println("Sensor 3 -> MUX: IR1=C10 IR2=C11 IR3=C12 IR4=C13 IR5=C14 (Goal Line)");
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

      // Determine if the servo motor is currently moving or settling.
      // This is used to block noise variance updates and temporarily apply a noise margin.
      bool isServoActive = (millis() - lastServoActionTime < SERVO_NOISE_WINDOW_MS);

      // Only adapt the environmental noise variance when the servo motor is inactive.
      // This prevents transient inductive noise from polluting the ambient noise baseline,
      // avoiding permanent desensitization while retaining environmental adaptivity.
      if (diff > 0 && !currentTriggerState[s][c] && !isServoActive) {
        noiseVarianceFloat[s][c] = (noiseVarianceFloat[s][c] * 0.99) + ((float)diff * 0.01);
      }

      // Calculate dynamic thresholds based on the calculated noise variance.
      // We do NOT clamp the threshold maximum, allowing full adaptivity to environment changes.
      int dynamicTrigger = max(30, (int)(noiseVarianceFloat[s][c] * 3.0));
      int dynamicRelease = max(20, (int)(noiseVarianceFloat[s][c] * 1.5));
      
      // Temporarily add a noise margin if the servo is active to prevent false triggers
      // caused by voltage sags and spikes from the motor during its movement window.
      if (isServoActive) {
        dynamicTrigger += 25;
        dynamicRelease += 15;
      }
      
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
      // Check only the front sensor row (Sensor 1, index 0) to detect ball entry.
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        // Detect shot starting on a clean rising edge on Sensor 1
        if (currentTriggerState[0][c] && !prevTriggerState[0][c]) {
          sensorsTriggeredInEvent[0][c] = true;
          shotDetected = true;
        }
      }
      
      // If a rising edge occurred on Sensor 1, start the shot tracking window
      if (shotDetected) {
        currentState = STATE_WAITING_FOR_GOAL;
        shotStartTime = millis();
        goalDetected = false;
        hasPredictedThisShot = false;
        
        // Set the first trigger timestamp (T+0ms) for Sensor 1 (index 0).
        // This marks the baseline time reference (T=0) for relative speed tracking.
        sensorRowFirstTriggerTime[0] = shotStartTime;
        
        // Chronological Log: Output S1 trigger event immediately at shot initiation (T+0ms)
        Serial.print("[T+0ms] S1 Input: [");
        for (int col = 0; col < CHANNELS_PER_SENSOR; col++) {
          Serial.print(sensorsTriggeredInEvent[0][col] ? "1" : "0");
          if (col < CHANNELS_PER_SENSOR - 1) Serial.print(",");
        }
        Serial.println("]");
        
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
          
          // Chronological Log: Output subsequent trigger events with relative timestamps at the exact millisecond they occur.
          unsigned long relTime = sensorRowFirstTriggerTime[s] - shotStartTime;
          Serial.print("[T+");
          Serial.print(relTime);
          Serial.print("ms] S");
          Serial.print(s + 1);
          Serial.print(" Input: [");
          for (int col = 0; col < CHANNELS_PER_SENSOR; col++) {
            Serial.print(sensorsTriggeredInEvent[s][col] ? "1" : "0");
            if (col < CHANNELS_PER_SENSOR - 1) Serial.print(",");
          }
          Serial.println("]");
        }
      }
      
      // Determine if Sensor 2 (middle row) has been triggered in this event window.
      // S2 is considered "gotten" when at least one of its channels is activated.
      bool s2_gotten = false;
      for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
        if (sensorsTriggeredInEvent[1][c]) {
          s2_gotten = true;
          break;
        }
      }
      
      // Make prediction as soon as Sensor 2 is gotten and we haven't predicted for this shot yet.
      // This ensures we have the trajectory data from both S1 and S2 to predict where S3 will be triggered.
      if (s2_gotten && !hasPredictedThisShot) {
        hasPredictedThisShot = true;
        
        // Pack S1 and S2 accumulated triggers into 5-bit masks.
        // Bits 0-4 correspond to channels 0-4 respectively.
        int s1_mask = 0;
        int s2_mask = 0;
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          if (sensorsTriggeredInEvent[0][c]) s1_mask |= (1 << c);
          if (sensorsTriggeredInEvent[1][c]) s2_mask |= (1 << c);
        }
        
        // Chronological Log: Output "Model Sent" equivalent indicating policy query state.
        unsigned long queryTime = millis() - shotStartTime;
        Serial.print("[T+");
        Serial.print(queryTime);
        Serial.print("ms] Query Policy State: S1=[");
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          Serial.print((s1_mask & (1 << c)) ? "1" : "0");
          if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
        }
        Serial.print("], S2=[");
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          Serial.print((s2_mask & (1 << c)) ? "1" : "0");
          if (c < CHANNELS_PER_SENSOR - 1) Serial.print(",");
        }
        Serial.println("]");
        
        // Perform O(1) zero-latency lookup in our local SRAM policy table.
        // Constrain indexes to prevent memory bound overflows.
        int s1_idx = constrain(s1_mask, 0, 31);
        int s2_idx = constrain(s2_mask, 0, 31);
        int predicted_pos = mcu_policy_table[s1_idx][s2_idx];
        unsigned long lookupEndTime = millis() - shotStartTime;
        
        if (predicted_pos != 255) {
          // Immediately command the goalkeeper servo to move to the predicted position.
          setGoalkeeperPosition(predicted_pos);
          Serial.print("[T+");
          Serial.print(lookupEndTime);
          Serial.print("ms] Local Inference: Keeper Pos = ");
          Serial.print(predicted_pos);
          Serial.println(" (took 0ms)");
        } else {
          // Fallback physics prediction in C++ if state has not been learned yet.
          // Extrapolates ball trajectory by calculating the centroids of active channels on S1 and S2.
          int c1 = -1, c2 = -1;
          
          long sumS1 = 0, countS1 = 0;
          for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
            if (s1_mask & (1 << c)) {
              sumS1 += c;
              countS1++;
            }
          }
          if (countS1 > 0) c1 = (int)round((double)sumS1 / countS1);
          
          long sumS2 = 0, countS2 = 0;
          for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
            if (s2_mask & (1 << c)) {
              sumS2 += c;
              countS2++;
            }
          }
          if (countS2 > 0) c2 = (int)round((double)sumS2 / countS2);
          
          int fallback_pos = 2;
          if (c1 != -1 && c2 != -1) {
            // First-order trajectory prediction: c3 = c2 + (c2 - c1) = 2*c2 - c1
            fallback_pos = constrain(2 * c2 - c1, 0, 4);
          } else if (c2 != -1) {
            fallback_pos = c2;
          }
          
          setGoalkeeperPosition(fallback_pos);
          Serial.print("[T+");
          Serial.print(lookupEndTime);
          Serial.print("ms] Fallback Physics Inference: Keeper Pos = ");
          Serial.print(fallback_pos);
          Serial.println(" (took 0ms)");
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
        // Pack S1, S2, and S3 accumulated triggers into 5-bit masks.
        int s1_mask = 0;
        int s2_mask = 0;
        int s3_mask = 0;
        for (int c = 0; c < CHANNELS_PER_SENSOR; c++) {
          if (sensorsTriggeredInEvent[0][c]) s1_mask |= (1 << c);
          if (sensorsTriggeredInEvent[1][c]) s2_mask |= (1 << c);
          if (sensorsTriggeredInEvent[2][c]) s3_mask |= (1 << c);
        }
        
        // Notify the MPU of the shot outcome using non-blocking RPC notify.
        // This prevents the MCU from blocking on MPU disk file I/O operations (saving history),
        // eliminating timeout warnings and keeping the main loop execution thread fully active.
        Bridge.notify("record_outcome", s1_mask, s2_mask, s3_mask, goalDetected, currentKeeperPos);
        Serial.println(">>> Shot outcome notified to MPU.");
        
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
        
        // Reset the goalkeeper to the neutral center position for the next shot.
        // Reset hasPredictedThisShot state flag so we can predict the next incoming shot.
        setGoalkeeperPosition(2);
        hasPredictedThisShot = false;
        
        currentState = STATE_WAITING_FOR_SHOT;
        Serial.println(">>> Cooldown complete. System ready for next shot.");
      }
      break;
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
