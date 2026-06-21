/*
 * sketch.ino — IR Tracker Sensor Test (Smartelex 5-Channel Tracker Sensor
 * Module)
 * ==============================================================================
 *
 * This diagnostic sketch runs on the STM32U585 microcontroller of the Arduino
 * Uno Q. It interfaces with a Smartelex 5-Channel Tracker Sensor
 * (TCRT5000-based reflection sensor), reading the 5 analog sensor outputs
 * (IR1-IR5) and printing their values to the App Lab console.
 *
 * Technical Specifications & Design Concepts:
 * -------------------------------------------
 * 1. Smartelex 5-Channel Tracker Sensor:
 *    - Operating Voltage: 3.3V – 5.0V DC. Connected to 5V.
 *    - Sensing element: TCRT5000 reflective optical sensor consisting of an
 * infrared emitter (GaAs diode) and an infrared phototransistor (silicon NPN)
 * side-by-side.
 *    - Output Type: 5 independent analog voltage outputs corresponding to the
 * intensity of reflected IR light.
 *    - Reflection Mechanics:
 *      - Higher Reflection (light surface / nearby ball) -> NPN phototransistor
 * conducts more -> collector voltage pulled lower -> analog voltage outputs a
 * LOWER value (near 0V).
 *      - Lower Reflection (black line / empty space) -> NPN phototransistor is
 * cut off -> collector voltage remains high -> analog voltage outputs a HIGHER
 * value (near VCC).
 *
 * 2. Wiring Interface (7 Wires):
 *    - Pin 1 (VCC)  → Connected to Arduino 5V pin (provides power to the IR
 * emitters and circuitry).
 *    - Pin 2 (GND)  → Connected to Arduino GND (common reference ground).
 *    - Pin 3 (IR1)  → Connected to analog pin A0 (Sensor channel 1 - leftmost).
 *    - Pin 4 (IR2)  → Connected to analog pin A1 (Sensor channel 2).
 *    - Pin 5 (IR3)  → Connected to analog pin A2 (Sensor channel 3 - center).
 *    - Pin 6 (IR4)  → Connected to analog pin A3 (Sensor channel 4).
 *    - Pin 7 (IR5)  → Connected to analog pin A4 (Sensor channel 5 -
 * rightmost).
 *
 * 3. Modular Architecture:
 *    - Encapsulated in the `FiveChannelAnalogTracker` class to guarantee
 * modularity and structure.
 */

// Include the Arduino Router Bridge library to enable serial output routing via
// MPU RPC.
#include <Arduino_RouterBridge.h>

/**
 * FiveChannelAnalogTracker Class
 * Encapsulates the operations of a 5-channel TCRT5000 infrared analog sensor
 * array. Provides modular methods to configure pins, read raw values, and
 * calculate reflection percentages.
 */
class FiveChannelAnalogTracker {
private:
  // Array of size 5 storing the MCU pin numbers mapped to each sensor output
  // (IR1-IR5).
  int sensorPins[5];

  // The maximum possible reading from the Analog-to-Digital Converter.
  // By default, the UNO Q's ADC runs in 10-bit mode, giving a maximum value of
  // 1023.
  int adcMaxVal;

public:
  /**
   * Constructor - Initialises pin assignments.
   * @param p1 Analog pin for Sensor 1 (IR1)
   * @param p2 Analog pin for Sensor 2 (IR2)
   * @param p3 Analog pin for Sensor 3 (IR3)
   * @param p4 Analog pin for Sensor 4 (IR4)
   * @param p5 Analog pin for Sensor 5 (IR5)
   */
  FiveChannelAnalogTracker(int p1, int p2, int p3, int p4, int p5) {
    sensorPins[0] = p1; // Map Pin for Channel 1 to private storage array.
    sensorPins[1] = p2; // Map Pin for Channel 2 to private storage array.
    sensorPins[2] = p3; // Map Pin for Channel 3 to private storage array.
    sensorPins[3] = p4; // Map Pin for Channel 4 to private storage array.
    sensorPins[4] = p5; // Map Pin for Channel 5 to private storage array.
    adcMaxVal = 1023;   // Default maximum ADC reading for a 10-bit conversion.
  }

  /**
   * begin - Configures GPIO pins for analog reading.
   * Sets pins to input mode.
   */
  void begin() {
    // Loop through all five pins to configure their input mode.
    for (int i = 0; i < 5; i++) {
      // Set pin mode to INPUT to allow analogRead without pullups interfering
      // with the analog voltage.
      pinMode(sensorPins[i], INPUT);
    }
  }

  /**
   * readRaw - Reads the raw analog state of a specific sensor channel.
   * Performs a dummy analog read followed by a 50 microsecond delay to allow
   * the internal sample-and-hold capacitor to settle before the second, true
   * read is executed. This is required to eliminate multiplexer
   * channel-switching crosstalk.
   * @param index 0-based sensor channel index (0 = leftmost IR1, 4 = rightmost
   * IR5).
   * @return int Raw ADC value (typically 0-1023).
   */
  int readRaw(int index) {
    // Bounds checking to avoid out-of-bounds memory reading of pin array.
    if (index < 0 || index >= 5) {
      // Return 0 if out of bounds to avoid memory fault.
      return 0;
    }
    // Perform a dummy analog read to switch the internal multiplexer to the
    // target channel.
    analogRead(sensorPins[index]);
    // Delay for 50 microseconds to allow the internal sample-and-hold capacitor
    // to charge fully from the sensor's output impedance.
    delayMicroseconds(50);
    // Query the MCU's ADC register for the settled voltage level.
    return analogRead(sensorPins[index]);
  }

  /**
   * getPercentageFromRaw - Calculates reflection percentage for a given raw ADC
   * value. 100% means highest reflection (sensor close to highly reflective
   * surface). 0% means lowest reflection (sensor over non-reflective surface or
   * dark line). Since lower analog values indicate higher reflection, we invert
   * the percentage calculation.
   * @param rawVal The raw ADC value (typically 0-1023).
   * @return float The computed reflection percentage.
   */
  float getPercentageFromRaw(int rawVal) {
    // Map raw value to fraction of ADC max: 0.0 (maximum reflection) to 1.0
    // (minimum reflection). Cast rawVal to float to avoid integer division
    // truncation before performing division.
    float fraction = (float)rawVal / adcMaxVal;
    // Invert fraction (so 0V/0 reading becomes 100% reflection, 1023 becomes 0%
    // reflection).
    float percentage = (1.0 - fraction) * 100.0;
    return percentage;
  }

  /**
   * getPercentage - Calculates reflection percentage for a channel by
   * performing a new read.
   */
  float getPercentage(int index) {
    // Bounds checking to avoid out-of-bounds memory reading of pin array.
    if (index < 0 || index >= 5) {
      return 0.0;
    }
    // Read raw analog value from the specified channel pin.
    int rawVal = readRaw(index);
    return getPercentageFromRaw(rawVal);
  }
};

// Global instance of FiveChannelAnalogTracker class initialized with pins:
// A0 (IR1), A1 (IR2), A2 (IR3), A3 (IR4), A4 (IR5) as specified in user manual.
FiveChannelAnalogTracker tracker(A0, A1, A2, A3, A4);

// [Static Thresholds Removed]
// We now use Adaptive Noise-Based Dynamic Thresholding.
// Each sensor continuously calculates its own ambient electrical/lighting noise floor (MAD)
// and mathematically generates perfectly scaled triggers.

// We no longer artificially limit the polling interval. We run non-blocking 
// to ensure the loop samples at maximum physical speed (approx 1500-2000 Hz).
// This guarantees we never miss a fast-moving ball transit.
// (SAMPLE_DELAY_MS has been removed)

// Moving average window size (set to 1 to eliminate group delay and capture instantaneous ball transits).
const int MOVING_AVERAGE_WINDOW_SIZE = 1;

// Circular buffer to store raw ADC histories for calculating moving averages.
int rawReadingsHistory[5][MOVING_AVERAGE_WINDOW_SIZE];

// Index slot in the rawReadingsHistory buffer indicating where to write the next sample.
// Ranges from 0 to 3 (circular buffer behavior) and is incremented at the end of each loop iteration.
int historyWriteIndex = 0;

// Declare a global array to preserve raw ADC values calibrated at startup (clear pitch state).
// This serves as the original reference baseline from when the application booted.
int calibrationBaseline[5] = {0, 0, 0, 0, 0};

// Declare a global array to maintain the running calibrated baseline as a floating-point value.
// Using single-precision float (32-bit IEEE 754) is required to prevent truncation and rounding errors
// during long-term dynamic updates, ensuring baseline drift tracks accurately.
float calibrationBaselineFloat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// Declare a global array to preserve the trigger state of each channel from the previous iteration.
// Tracking the previous state is required to execute state-transition detection (edge triggering).
bool prevTriggerState[5] = {false, false, false, false, false};

// Declare an array to track if a sensor's baseline has fallen significantly behind the signal
// (e.g., due to thermal drift or a shadow clearing) and needs to fast-adapt all the way to the center.
bool isRecovering[5] = {false, false, false, false, false};

// Declare an array to track the exact millisecond timestamp when each sensor enters the triggered state.
// This is required to calculate the transit duration of the object and filter out slow entities like hands.
unsigned long triggerStartTime[5] = {0, 0, 0, 0, 0};

// Declare an array to track the strongest reflection (lowest raw ADC value) during a trigger event.
// This allows us to report the peak intensity of the ball pass upon completion of the event.
int minRawDuringTrigger[5] = {1023, 1023, 1023, 1023, 1023};

// Declare a global array to store the filtered analog readings for each of the 5 channels.
// Using float types prevents accumulation of rounding errors during the average calculations.
float filteredReadings[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float prevFilteredReadings[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// Declare an array to continuously track the electrical variance (noise floor) for each sensor.
// This allows the dynamic thresholding algorithm to perfectly tune sensitivity in real-time.
float noiseVarianceFloat[5] = {5.0, 5.0, 5.0, 5.0, 5.0};

// Declare arrays to hold the dynamically calculated thresholds for diagnostic output.
int dynamicTriggerThresholds[5] = {30, 30, 30, 30, 30};
int dynamicReleaseThresholds[5] = {20, 20, 20, 20, 20};

// Global flags to aggregate multiple sensor triggers into a single unified ML event array.
// This ensures that if a ball rolls over two sensors simultaneously, they are output as a single vector.
bool sensorsTriggeredInEvent[5] = {false, false, false, false, false};
unsigned long lastEventPrintTime = 0; // Cooldown timer to prevent multiple prints for the same ball
const unsigned long EVENT_COOLDOWN_MS = 500; // 500ms blind period after a shot

/**
 * setup() - Microcontroller initialisation routine.
 * Executed once when the board is booted or reset.
 */
void setup() {
  // Initialise virtual serial transceiver (baud rate is ignored by RPC driver but set to 9600 for App Lab UI).
  Serial.begin(9600);

  // Initialise the Router Bridge module to establish virtual serial interface
  // to App Lab.
  Bridge.begin();

  // Call the begin method on our tracker object to configure analog pins.
  tracker.begin();

  // Print pre-calibration notification. This notifies the user via the virtual
  // serial port that the board is initializing and that the pitch must be kept
  // clear of objects.
  Serial.println("Stabilizing hardware components... Keep pitch clear!");

  // Pause execution for 2000 milliseconds to allow power rails and IR emitters
  // to stabilize. The Smartelex 5-Channel Tracker's TCRT5000 GaAs diodes and
  // NPN phototransistors need time to reach thermal and optical equilibrium,
  // and the MCU's internal analog reference voltage requires settling time to
  // ensure accurate subsequent analog-to-digital conversions.
  delay(2000);

  // Seed the circular buffer history with the initial raw readings from each channel.
  // This initializes the moving average history to prevent startup transient spikes.
  for (int i = 0; i < 5; i++) {
    // Read the current raw ADC level from channel index i.
    int initialVal = tracker.readRaw(i);
    // Fill all window slots in the circular history buffer with the initial raw value.
    for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
      // Assign the raw value to the history slot for sensor i.
      rawReadingsHistory[i][sample] = initialVal;
    }
  }

  // Settle the circular history buffer by running the read loop for 20 cycles (40 ms).
  // This fills the buffer with real, active samples before calibration.
  for (int sample = 0; sample < 20; sample++) {
    // Loop through all 5 channels to write active readings into the buffer.
    for (int i = 0; i < 5; i++) {
      // Store the active raw reading in the buffer write slot.
      rawReadingsHistory[i][historyWriteIndex] = tracker.readRaw(i);
    }
    // Increment the write index in circular fashion.
    historyWriteIndex = (historyWriteIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;
    // Wait 2 milliseconds to simulate the loop timing interval.
    delay(2);
  }

  // Calculate the moving average of the history buffer to establish the startup baseline.
  for (int i = 0; i < 5; i++) {
    // Accumulator variable to hold the sum of the history buffer.
    long sum = 0;
    // Loop through all window slots to sum the history buffer elements.
    for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
      // Add the sample value to the sum accumulator.
      sum += rawReadingsHistory[i][sample];
    }
    // Calculate the mathematical average float.
    float average = (float)sum / MOVING_AVERAGE_WINDOW_SIZE;
    // Assign the running float baseline tracking array to the computed average.
    calibrationBaselineFloat[i] = average;
    // Cast the float average to an integer for quick runtime comparisons.
    calibrationBaseline[i] = (int)average;
  }

  // Print the calibrated baseline readings for diagnostic purposes.
  // This allows the operator to verify that the start reference is correct.
  Serial.print("Calibrated Baselines: ");
  for (int i = 0; i < 5; i++) {
    Serial.print("IR");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(calibrationBaseline[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Initialize all previous trigger states to false.
  // This assumes the sensors start in an untriggered state (no object present).
  for (int i = 0; i < 5; i++) {
    // Set state for channel i to false.
    prevTriggerState[i] = false;
  }

  // Print introductory diagnostic banner to App Lab console using print
  // commands.
  Serial.println("=================================================");
  Serial.println("  5-CHANNEL ANALOG IR TRACKER SENSOR TEST (EDGE)");
  Serial.println("=================================================");
  Serial.println("Wiring Pinout (Sensor -> Arduino UNO Q):");
  Serial.println("  VCC (Power)  -> 5V Pin");
  Serial.println("  GND (Ground) -> GND Pin");
  Serial.println("  IR1 (Left)   -> Pin A0");
  Serial.println("  IR2 (Mid-L)  -> Pin A1");
  Serial.println("  IR3 (Center) -> Pin A2");
  Serial.println("  IR4 (Mid-R)  -> Pin A3");
  Serial.println("  IR5 (Right)  -> Pin A4");
  Serial.println("=================================================");
  Serial.println("Format: IR1[Raw|%Reflect] IR2[Raw|%Reflect] ... IR5");
  Serial.println("=================================================");
}

/**
 * loop() - Continuous application runtime routine.
 * Executed repeatedly after setup completes.
 */
void loop() {
  // Static variable to track the last time we printed a heartbeat diagnostic message.
  static unsigned long lastDebugPrint = 0;

  // Declare a local array to hold the current raw analog readings of the 5
  // channels. Allocating on the stack allows fast local access during the
  // comparisons.
  int currentRawReadings[5];

  // Declare a local boolean array to hold the current trigger state of the 5
  // channels. Each slot is true if the corresponding channel's filtered value
  // deviates from its baseline.
  bool currentTriggerState[5];

  // Loop through all 5 channels to read, filter, and evaluate trigger states.
  for (int i = 0; i < 5; i++) {
    // Query the tracker to read the analog voltage value on channel i and store it locally.
    currentRawReadings[i] = tracker.readRaw(i);

    // Write the new raw reading into the active circular buffer index slot for channel i.
    rawReadingsHistory[i][historyWriteIndex] = currentRawReadings[i];

    // Compute the sum of the history buffer to calculate the moving average.
    long sum = 0;
    // Loop through the circular buffer slots to accumulate all samples.
    for (int sample = 0; sample < MOVING_AVERAGE_WINDOW_SIZE; sample++) {
      // Add the history value to the sum accumulator.
      sum += rawReadingsHistory[i][sample];
    }
    // Calculate the moving average.
    filteredReadings[i] = (float)sum / MOVING_AVERAGE_WINDOW_SIZE;

    // Calculate signal deviation. A positive difference indicates a reflective object (ball).
    int diff = calibrationBaseline[i] - (int)filteredReadings[i];

    // Evaluate Schmitt trigger state transition using dynamic hysteresis thresholds.
    // Evaluated before noise tracking to ensure ball strikes are mathematically excluded from the noise envelope.
    bool newState = prevTriggerState[i];
    if (!prevTriggerState[i]) {
      if (diff >= dynamicTriggerThresholds[i]) newState = true;
    } else {
      if (diff < dynamicReleaseThresholds[i]) newState = false;
    }
    currentTriggerState[i] = newState;

    // Track pure ambient noise envelope using Exponential Moving Average.
    // Only sub-threshold positive deviations are sampled.
    if (diff > 0 && !currentTriggerState[i]) {
      noiseVarianceFloat[i] = (noiseVarianceFloat[i] * 0.99) + ((float)diff * 0.01);
    }

    // Calculate dynamic thresholds based on the learned noise envelope.
    int dynamicTrigger = max(30, (int)(noiseVarianceFloat[i] * 3.0));
    int dynamicRelease = max(20, (int)(noiseVarianceFloat[i] * 1.5));
    dynamicTriggerThresholds[i] = dynamicTrigger;
    dynamicReleaseThresholds[i] = dynamicRelease;

    // --- Sub-Threshold Centered Baseline Adaptation ---
    int upwardDiff = filteredReadings[i] - calibrationBaselineFloat[i];
    
    // State Machine: If the baseline falls behind the signal (e.g. thermal drift or shadow cleared), 
    // enter a recovery state to fast-adapt perfectly to the center of the new ambient signal.
    if (upwardDiff > dynamicTriggerThresholds[i]) {
      isRecovering[i] = true;
    }
    // Exit the recovery state once the baseline is fully centered.
    if (upwardDiff <= 0) {
      isRecovering[i] = false;
    }

    if (isRecovering[i]) {
      // Fast upward adaptation to recover from environmental shifts.
      calibrationBaselineFloat[i] = (calibrationBaselineFloat[i] * 0.9) + (filteredReadings[i] * 0.1);
    } else {
      if (currentTriggerState[i] && (millis() - triggerStartTime[i] > 500)) {
        // Fast downward adaptation to clear latch-up states from permanent physical obstructions (e.g. hands).
        calibrationBaselineFloat[i] = (calibrationBaselineFloat[i] * 0.9) + (filteredReadings[i] * 0.1);
      } else {
        // Slow, symmetric adaptation to keep the baseline perfectly centered in the ambient noise wave.
        calibrationBaselineFloat[i] = (calibrationBaselineFloat[i] * 0.999) + (filteredReadings[i] * 0.001);
      }
    }
    
    // Sync integer baseline for the next loop iteration.
    calibrationBaseline[i] = (int)calibrationBaselineFloat[i];
  }

  // ZERO-LATENCY RISING EDGE DETECTION
  // Trigger instantly upon beam break to satisfy the 25ms physical ball transit constraint.
  // We must trigger instantly the exact millisecond any sensor is breached.
  
  bool triggerFiredThisLoop = false;
  
  // We only evaluate new triggers if we are past the 500ms cooldown from the last shot.
  // This prevents a single ball pass from printing multiple arrays if it rolls over multiple sensors.
  if (millis() - lastEventPrintTime > EVENT_COOLDOWN_MS) {
    for (int i = 0; i < 5; i++) {
      if (currentTriggerState[i]) {
        if (!prevTriggerState[i]) {
          // RISING EDGE: The sensor was breached this exact microsecond.
          // We also update triggerStartTime here in case the baseline logic needs it.
          triggerStartTime[i] = millis();
          
          sensorsTriggeredInEvent[i] = true;
          triggerFiredThisLoop = true;
        }
      }
    }
    
    // If ANY sensor was breached during this loop iteration, we print the ML array instantly.
    // 0ms delay. Absolute maximum physical speed.
    if (triggerFiredThisLoop) {
      Serial.print("[");
      for (int i = 0; i < 5; i++) {
        Serial.print(sensorsTriggeredInEvent[i] ? "1" : "0");
        if (i < 4) Serial.print(", ");
        
        // Reset the flag immediately
        sensorsTriggeredInEvent[i] = false;
      }
      Serial.println("]");
      
      // Engage the 500ms cooldown so we don't spam arrays as the ball finishes passing.
      lastEventPrintTime = millis();
    }
  } else {
    // If we are in the cooldown period, we still need to record start times for baseline logic
    // in case a shadow triggers a sensor so the 500ms shadow latch-up clearance works.
    for (int i = 0; i < 5; i++) {
      if (currentTriggerState[i] && !prevTriggerState[i]) {
        triggerStartTime[i] = millis();
      }
    }
  }

  // Non-blocking 1-second diagnostic heartbeat to output live state.
  if (millis() - lastDebugPrint > 1000) {
    lastDebugPrint = millis();
    Serial.print("HEARTBEAT | Baselines: [");
    for (int i = 0; i < 5; i++) {
      Serial.print(calibrationBaseline[i]);
      if (i < 4) Serial.print(", ");
    }
    Serial.print("] | Raw: [");
    for (int i = 0; i < 5; i++) {
      Serial.print(currentRawReadings[i]);
      if (i < 4) Serial.print(", ");
    }
    Serial.print("] | DynThresh: [");
    for (int i = 0; i < 5; i++) {
      Serial.print(dynamicTriggerThresholds[i]);
      if (i < 4) Serial.print(", ");
    }
    Serial.println("]");
  }

  // Loop to update the baseline buffer for the next iteration's comparison.
  // Copying the current trigger states to the prevTriggerState buffer allows
  // edge detection on the next cycle.
  for (int i = 0; i < 5; i++) {
    // Assign the current channel's trigger state to the historical memory slot.
    prevTriggerState[i] = currentTriggerState[i];
    // Assign the current filtered reading to the previous buffer for derivative calculation.
    prevFilteredReadings[i] = filteredReadings[i];
  }

  // Increment the circular buffer write index for the next loop iteration.
  // The index is wrapped around using modulo MOVING_AVERAGE_WINDOW_SIZE to maintain buffer boundary logic.
  historyWriteIndex = (historyWriteIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;

  // We must yield a tiny amount of time (2ms) at the end of the loop to allow the 
  // underlying RTOS and RouterBridge to flush the virtual serial buffer.
  // Without this delay, the tight loop starves the background RPC tasks, 
  // crashing the bridge and preventing ANY serial prints from reaching the console.
  delay(2);
}
