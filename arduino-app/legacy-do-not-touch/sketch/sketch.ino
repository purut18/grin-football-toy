/*
 * sketch.ino — GRIN Football Toy MCU Firmware (STM32U585 via Zephyr RTOS)
 * =========================================================================
 *
 * This Arduino sketch runs on the STM32U585 microcontroller of the Arduino
 * UNO Q. It handles all real-time hardware I/O for the football desk toy:
 *
 *   1. Reading 15 IR proximity sensors (3 rows × 5 columns) on the pitch
 *   2. Controlling a servo motor for goalkeeper lateral movement
 *   3. Exposing sensor data and servo control as RPC functions to the MPU
 *
 * The MPU (Qualcomm QRB2210) runs the Python-based Q-learning AI that
 * processes sensor data and decides the goalkeeper's position. This sketch
 * acts as the hardware abstraction layer — it reads raw sensor values and
 * moves the servo, while all intelligence lives on the MPU side.
 *
 * Communication Architecture:
 *   The Arduino_RouterBridge library provides bidirectional RPC between
 *   this MCU sketch and the MPU's Python scripts. The arduino-router
 *   service (a Linux daemon) acts as the traffic controller, forwarding
 *   MessagePack RPC calls over the internal /dev/ttyHS1 serial bus.
 *
 *   MCU exposes two RPC functions to MPU:
 *     - "get_sensors"    → returns uint16_t bitmask of all 15 sensors
 *     - "set_keeper_pos" → accepts int (0-4), moves servo to position
 *
 * Sensor Bitmask Encoding:
 *   The 15 sensor readings are packed into a single uint16_t for efficient
 *   RPC transmission. Bit layout (LSB → MSB):
 *     Bits  0-4:  Row 1 (Front)   — first detection zone
 *     Bits  5-9:  Row 2 (Middle)  — approach zone before goalkeeper
 *     Bits 10-14: Row 3 (Goal)    — behind goalkeeper, detects goals
 *   Bit = 1 means ball detected, Bit = 0 means no ball.
 *
 * Pin Assignments:
 *   Row 1 (Front):  D2, D4, D5, D6, D7
 *   Row 2 (Middle): D8, D10, D11, D12, D13
 *   Row 3 (Goal):   A0, A1, A2, A3, A4
 *   Servo:          D3 (PWM-capable via Servo library timer)
 *
 * Hardware Notes:
 *   - IR sensors are active-LOW: they pull their output pin LOW when an
 *     object (the ball) is detected via reflected infrared light.
 *   - The servo operates via the Servo library, which uses internal timers
 *     to generate the required 50Hz PWM signal independently of the
 *     board's analogWrite PWM frequency (500Hz on UNO Q).
 *   - Bridge.provide_safe() is used instead of Bridge.provide() because
 *     servo control and digitalRead use Arduino APIs that are not safe
 *     to call from the background RPC thread. provide_safe() defers
 *     execution to the main loop() context where these APIs are safe.
 *
 * Dependencies:
 *   - Arduino_RouterBridge.h: Bridge RPC library for MPU↔MCU communication
 *   - Servo.h: Standard Arduino servo control library (timer-based PWM)
 */

/* =========================================================================
 * LIBRARY INCLUDES
 * ========================================================================= */

/*
 * Arduino_RouterBridge.h — Bridge RPC Library
 * Provides the Bridge singleton object for registering RPC functions
 * (provide/provide_safe) and calling MPU-side functions (call/notify).
 * Communication flows through the arduino-router Linux service via
 * the /dev/ttyHS1 serial interface using MessagePack serialisation.
 */
#include <Arduino_RouterBridge.h>

/*
 * Servo.h — Standard Arduino Servo Control Library
 * Controls hobby servo motors by generating precise 50Hz PWM signals
 * using hardware timers. Unlike analogWrite (which runs at 500Hz on
 * UNO Q and is unsuitable for servos), the Servo library manages its
 * own timer-based pulse generation at the correct 50Hz frequency.
 * Pulse width range: 544µs (0°) to 2400µs (180°).
 */
#include <Servo.h>


/* =========================================================================
 * PIN DEFINITIONS — IR SENSOR GRID (3 rows × 5 columns)
 * =========================================================================
 *
 * Physical Layout (top-down view, ball travels from top to bottom):
 *
 *   [Shooter End]
 *   ┌─────────────────────────────────────┐
 *   │  D2    D4    D5    D6    D7         │  ← ROW 1 (Front)
 *   │                                     │
 *   │  D8    D10   D11   D12   D13        │  ← ROW 2 (Middle)
 *   │                                     │
 *   │         ████ KEEPER [D3] ████       │  ← Servo on D3
 *   │                                     │
 *   │  A0    A1    A2    A3    A4         │  ← ROW 3 (Goal Line)
 *   └─────────────────────────────────────┘
 *   [Goal Net]
 *
 * Note: D3 is used for the servo (PWM capable). D9 is skipped because
 * it is also PWM-capable and may be needed for future features.
 */

/* Number of sensors per row — matches the 5-column pitch layout */
const int SENSORS_PER_ROW = 5;

/*
 * ROW 1 (Front) — First detection zone.
 * These sensors detect the ball as it enters the pitch from the shooter.
 * Connected to digital pins D2, D4, D5, D6, D7.
 * (D3 is reserved for the servo motor.)
 */
const int ROW1_PINS[SENSORS_PER_ROW] = {D2, D4, D5, D6, D7};

/*
 * ROW 2 (Middle) — Approach zone.
 * These sensors detect the ball approaching the goalkeeper.
 * The AI uses readings from both Row 1 and Row 2 to determine
 * the ball's trajectory (which column it's heading towards).
 * Connected to digital pins D8, D10, D11, D12, D13.
 * (D9 is reserved for potential future PWM use.)
 */
const int ROW2_PINS[SENSORS_PER_ROW] = {D8, D10, D11, D12, D13};

/*
 * ROW 3 (Goal Line) — Goal detection zone.
 * These sensors are positioned behind the goalkeeper. If any of
 * these sensors detect the ball, a goal has been scored. The column
 * information tells the AI WHERE the goal was scored for training.
 * Connected to analog pins A0-A4 used as digital inputs.
 */
const int ROW3_PINS[SENSORS_PER_ROW] = {A0, A1, A2, A3, A4};


/* =========================================================================
 * SERVO CONFIGURATION
 * ========================================================================= */

/*
 * SERVO_PIN — Digital pin connected to the servo signal wire.
 * D3 is used because it is PWM-capable, though the Servo library
 * uses its own timer-based PWM generation rather than analogWrite.
 */
const int SERVO_PIN = D3;

/*
 * Servo angle range for goalkeeper positions 0-4.
 * Position 0 = far left (SERVO_MIN_ANGLE degrees).
 * Position 4 = far right (SERVO_MAX_ANGLE degrees).
 * Position 2 = centre (90 degrees).
 */
const int SERVO_MIN_ANGLE = 0;    /* Angle in degrees for position 0 (far left) */
const int SERVO_MAX_ANGLE = 180;  /* Angle in degrees for position 4 (far right) */
const int KEEPER_POSITIONS = 5;   /* Total discrete positions matching sensor columns */

/*
 * Servo object — manages the timer-based PWM pulse generation.
 * The Servo library allocates a hardware timer to produce precise
 * pulse widths that correspond to the desired shaft angle.
 */
Servo goalkeeper;


/* =========================================================================
 * SENSOR DATA STORAGE
 * =========================================================================
 *
 * The latest sensor readings are stored in a volatile uint16_t bitmask.
 * This is updated continuously in loop() and read by the RPC handler.
 *
 * volatile keyword tells the compiler not to optimise away reads of this
 * variable, since it can change asynchronously (written in loop(), read
 * by the RPC background thread via provide_safe callback).
 */
volatile uint16_t sensorBitmask = 0;


/* =========================================================================
 * RPC CALLBACK FUNCTIONS
 * =========================================================================
 *
 * These functions are exposed to the MPU (Linux) side via Bridge.provide_safe().
 * The MPU calls them by name using Bridge.call("function_name", args).
 *
 * provide_safe() ensures these callbacks execute in the main loop() context,
 * which is necessary because they use Arduino APIs (digitalRead, Servo.write)
 * that are not thread-safe and must not be called from the background
 * RPC thread.
 */

/**
 * getSensorData — RPC handler for "get_sensors"
 *
 * Called by the MPU to retrieve the current state of all 15 IR sensors.
 * Returns the pre-computed bitmask that was most recently updated by
 * readAllSensors() in the main loop.
 *
 * @return uint16_t — Packed 15-bit sensor bitmask.
 *         Bits  0-4:  Row 1 (front)  sensors
 *         Bits  5-9:  Row 2 (middle) sensors
 *         Bits 10-14: Row 3 (goal)   sensors
 *         Bit = 1: ball detected, Bit = 0: no ball.
 */
uint16_t getSensorData() {
    return sensorBitmask;
}

/**
 * setGoalkeeperPosition — RPC handler for "set_keeper_pos"
 *
 * Called by the MPU to command the goalkeeper's lateral position.
 * Accepts a position index (0-4) and maps it to a servo angle.
 *
 * Position-to-angle mapping (linear interpolation):
 *   Position 0 → 0°   (far left)
 *   Position 1 → 45°  (left)
 *   Position 2 → 90°  (centre)
 *   Position 3 → 135° (right)
 *   Position 4 → 180° (far right)
 *
 * @param position int — Desired keeper position (0-4).
 *                       Values outside this range are clamped.
 */
void setGoalkeeperPosition(int position) {
    /*
     * Clamp the position to valid range [0, 4] to prevent servo damage
     * from out-of-range angle commands. This is a safety measure against
     * erroneous RPC calls from the MPU.
     */
    position = constrain(position, 0, KEEPER_POSITIONS - 1);

    /*
     * map() performs linear interpolation:
     *   output = (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
     *
     * For position=2: angle = (2-0) * (180-0) / (4-0) + 0 = 90°
     */
    int angle = map(position, 0, KEEPER_POSITIONS - 1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    /*
     * Command the servo to the calculated angle.
     * The Servo library converts the angle to a pulse width:
     *   0°   → 544µs pulse
     *   90°  → 1472µs pulse
     *   180° → 2400µs pulse
     * These pulses are generated at 50Hz (20ms period) by the timer.
     */
    goalkeeper.write(angle);
}


/* =========================================================================
 * SENSOR READING FUNCTION
 * ========================================================================= */

/**
 * readAllSensors — Read all 15 IR sensors and pack into bitmask
 *
 * Iterates through all 3 rows × 5 columns of IR proximity sensors,
 * reads each digital pin, and packs the results into a uint16_t bitmask.
 *
 * IR Sensor Behaviour:
 *   Most IR proximity/obstacle sensors are ACTIVE LOW — they pull their
 *   output pin LOW when an object is detected (reflected IR light received).
 *   When no object is present, the output is HIGH (pulled up internally).
 *
 *   Therefore: digitalRead() == LOW → ball detected → bit = 1
 *              digitalRead() == HIGH → no ball     → bit = 0
 *
 * Bitmask Packing:
 *   Each sensor's detection state is stored as a single bit.
 *   Row 1 sensors occupy bits 0-4  (offset 0).
 *   Row 2 sensors occupy bits 5-9  (offset 5).
 *   Row 3 sensors occupy bits 10-14 (offset 10).
 *
 *   The bitwise OR (|=) sets the appropriate bit, and the left shift
 *   (<<) positions each sensor's bit at the correct offset.
 */
void readAllSensors() {
    /* Start with a clean bitmask — all bits cleared (no detections) */
    uint16_t newBitmask = 0;

    for (int i = 0; i < SENSORS_PER_ROW; i++) {
        /*
         * Row 1 (Front) — bits 0-4
         * digitalRead returns LOW (0) when ball detected by IR sensor.
         * We invert the logic: LOW → set bit (ball present).
         */
        if (digitalRead(ROW1_PINS[i]) == LOW) {
            newBitmask |= (1 << i);           /* Set bit at position i (0-4) */
        }

        /*
         * Row 2 (Middle) — bits 5-9
         * Same active-LOW logic as Row 1.
         * Bit offset by 5 to pack into bits 5-9 of the bitmask.
         */
        if (digitalRead(ROW2_PINS[i]) == LOW) {
            newBitmask |= (1 << (i + 5));     /* Set bit at position i+5 (5-9) */
        }

        /*
         * Row 3 (Goal Line) — bits 10-14
         * Same active-LOW logic as Row 1.
         * Bit offset by 10 to pack into bits 10-14 of the bitmask.
         */
        if (digitalRead(ROW3_PINS[i]) == LOW) {
            newBitmask |= (1 << (i + 10));    /* Set bit at position i+10 (10-14) */
        }
    }

    /*
     * Atomically update the shared bitmask variable.
     * Although Arduino runs single-threaded in loop(), the provide_safe()
     * callback executes between loop() iterations via __loopHook,
     * so this assignment is effectively atomic for our use case.
     */
    sensorBitmask = newBitmask;
}


/* =========================================================================
 * ARDUINO LIFECYCLE FUNCTIONS
 * ========================================================================= */

/**
 * setup — Arduino initialisation function (runs once at boot)
 *
 * Configures all hardware peripherals (GPIO pins, servo, serial) and
 * initialises the Bridge RPC connection to the MPU. Registers the two
 * RPC callback functions that the MPU will call during gameplay.
 *
 * Execution Order:
 *   1. Serial initialisation (for debug output to App Lab console)
 *   2. IR sensor pin configuration (all 15 pins as INPUT_PULLUP)
 *   3. Servo initialisation (attach to pin, move to centre)
 *   4. Bridge RPC initialisation (connect to arduino-router)
 *   5. RPC function registration (expose get_sensors & set_keeper_pos)
 */
void setup() {
    /*
     * Initialise Serial communication at 9600 baud.
     * On UNO Q, Serial prints to the Arduino App Lab console
     * via the Bridge RPC mechanism (not hardware UART).
     * This is useful for debug output during development.
     */
    Serial.begin(9600);
    Serial.println("[MCU] GRIN Football Toy — MCU Firmware Initialising...");

    /*
     * Configure all 15 IR sensor pins as digital inputs with internal
     * pull-up resistors enabled.
     *
     * INPUT_PULLUP activates the STM32's internal pull-up resistor
     * (~40kΩ to VDD). This ensures a defined HIGH state when no
     * external device is driving the pin, preventing floating inputs
     * that would cause erratic readings.
     *
     * The IR sensors have open-collector or push-pull outputs that
     * drive LOW when triggered, overriding the pull-up.
     */
    for (int i = 0; i < SENSORS_PER_ROW; i++) {
        /* Row 1 (Front) sensor pins — D2, D4, D5, D6, D7 */
        pinMode(ROW1_PINS[i], INPUT_PULLUP);

        /* Row 2 (Middle) sensor pins — D8, D10, D11, D12, D13 */
        pinMode(ROW2_PINS[i], INPUT_PULLUP);

        /* Row 3 (Goal Line) sensor pins — A0, A1, A2, A3, A4
         * (analog pins used as digital inputs) */
        pinMode(ROW3_PINS[i], INPUT_PULLUP);
    }
    Serial.println("[MCU] 15 IR sensor pins configured (INPUT_PULLUP).");

    /*
     * Initialise the servo motor.
     * attach() allocates a hardware timer for PWM generation and
     * begins outputting pulses on the specified pin.
     * write(90) moves the servo to centre position (90°).
     */
    goalkeeper.attach(SERVO_PIN);
    goalkeeper.write(90); /* Centre position — equal coverage of all columns */
    Serial.println("[MCU] Servo attached to D3, centred at 90°.");

    /*
     * Initialise the Bridge RPC connection.
     * Bridge.begin() establishes the serial transport to the
     * arduino-router Linux service. This must succeed before any
     * RPC calls can be made or received.
     *
     * If Bridge.begin() fails (returns false), the MCU enters an
     * infinite loop since it cannot function without RPC.
     */
    if (!Bridge.begin()) {
        Serial.println("[MCU] ERROR: Bridge.begin() failed! Halting.");
        while (true) {
            /* Halt execution — the MCU cannot proceed without the Bridge.
             * This typically indicates the arduino-router service is not
             * running on the MPU. Restart it with:
             *   sudo systemctl restart arduino-router
             */
            delay(1000);
        }
    }
    Serial.println("[MCU] Bridge RPC connected to arduino-router.");

    /*
     * Register RPC callback functions with the Bridge.
     *
     * provide_safe() is used instead of provide() because our callbacks
     * use Arduino APIs (digitalRead, Servo.write) that are NOT safe to
     * call from the background RPC thread. provide_safe() defers
     * callback execution to the main loop() context via __loopHook,
     * ensuring thread safety.
     *
     * Function names ("get_sensors", "set_keeper_pos") are the identifiers
     * the MPU uses in Bridge.call("get_sensors") / Bridge.call("set_keeper_pos", pos).
     */
    Bridge.provide_safe("get_sensors", getSensorData);
    Bridge.provide_safe("set_keeper_pos", setGoalkeeperPosition);
    Serial.println("[MCU] RPC functions registered: get_sensors, set_keeper_pos");

    Serial.println("[MCU] Initialisation complete. Ready for play!");
}

/**
 * loop — Arduino main loop (runs continuously after setup)
 *
 * Continuously reads all 15 IR sensors and updates the shared bitmask.
 * The polling rate is ~100Hz (10ms delay), which is faster than the
 * MPU's 50Hz polling rate, ensuring the MPU always gets fresh data.
 *
 * The Bridge RPC callbacks (getSensorData, setGoalkeeperPosition) are
 * executed between loop() iterations by the provide_safe() mechanism,
 * which hooks into the Zephyr RTOS loop scheduler.
 */
void loop() {
    /*
     * Read all 15 IR sensors and update the sensorBitmask variable.
     * This runs at ~100Hz, providing fresh data for the MPU's
     * ~50Hz RPC polling rate.
     */
    readAllSensors();

    /*
     * 10ms delay between sensor reads.
     * This gives a ~100Hz sampling rate, which is more than sufficient
     * for tracking a ball rolling at desk-toy speeds (~0.5-2 m/s).
     * At 100Hz, we get a new reading every 10ms = every ~5-20mm of
     * ball travel at typical play speeds.
     *
     * The delay also yields CPU time to the Zephyr RTOS scheduler,
     * allowing the Bridge RPC background thread and provide_safe()
     * callbacks to execute.
     */
    delay(10);
}
