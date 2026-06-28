"""
main.py — Production Dual IR Sensor Tracker (MPU Orchestrator App)
==================================================================

This script runs on the MPU (Qualcomm QRB2210 microprocessor) of the Arduino Uno Q.
It initializes the App Lab framework runtime environment and enters an idle loop,
allowing the MCU sketch (running on the STM32U585) to perform all sensor reading,
MUX control, signal processing, and console printing independently.

Technical Specifications:
-------------------------
1. MPU-MCU Interface:
   - Communication channel is governed by the arduino-router service daemon
     running on the MPU's embedded Linux (Yocto/OpenWrt-based).
   - The MPU initiates and maintains the application daemon process lifecycle.
     When this script is launched by App Lab, it signals the arduino-router to
     also start/restart the MCU sketch on the STM32U585.
   - The MCU sketch uses the Arduino_RouterBridge RPC library to serialize
     Serial.print() calls into RPC messages. These messages are routed through
     the shared-memory IPC channel to the MPU, which presents them in the
     App Lab CLI/console output.

2. Architecture:
   - Follows the Arduino App Lab framework convention where every app must have
     a python/main.py entry point that calls App.run() with a user_loop callback.
   - Implements a class-based application manager pattern (GrinFootballApp) to
     hold application state and run loops.
   - The user_loop is intentionally idle (sleep-only) because all real-time
     sensor processing happens on the MCU side. The MPU's role is purely to
     keep the App Lab process alive and relay serial output.

3. Hardware Stack (managed by MCU sketch, not this file):
   - 2x Smartelex 5-Channel Tracker Sensors (TCRT5000-based)
   - 1x SmartElex CD74HC4067 16-channel analog/digital MUX breakout board
   - Arduino Uno Q (STM32U585 MCU + QRB2210 MPU)
"""

# Import the App class from arduino.app_utils which provides the runtime loop,
# signal handling (SIGTERM/SIGINT for graceful shutdown), and process lifecycle
# management. This is the mandatory entry point API for all App Lab applications.
from arduino.app_utils import App

# Import the time module to provide standard delay/sleep functions.
# Used to suspend the MPU's Python process between idle loop iterations,
# reducing CPU core consumption on the QRB2210.
import time


class GrinFootballApp:
    """
    GrinFootballApp
    Orchestrates the lifecycle of the MPU-side production application.

    This class maintains an idle state to allow the MCU sketch to execute
    its real-time dual IR sensor tracking pipeline independently. The MPU
    does not participate in sensor reading or signal processing — it only
    keeps the App Lab daemon alive so that:
      1. The MCU sketch continues running (App Lab kills the sketch when
         the MPU process exits).
      2. Serial output from the MCU (via RouterBridge RPC) is relayed to
         the App Lab console for operator monitoring and ML data collection.
    """

    def __init__(self):
        """
        Constructor — Initializes application state.
        In this production app, no MPU-side state is required because all
        sensor logic runs on the MCU. This constructor is a placeholder
        for future MPU-side features (e.g., BLE broadcasting, data logging,
        or RL model inference offloading to the MPU's Linux environment).
        """
        # No state initialization needed for the current production configuration.
        pass

    def run_loop(self):
        """
        run_loop — The main execution cycle called by the App Lab framework.

        This method is invoked repeatedly by App.run() in an infinite loop.
        Each invocation suspends MPU execution for 5 seconds to conserve
        processor cycles. The 5-second interval is long enough to avoid
        wasting CPU but short enough that the App Lab framework's internal
        watchdog timer does not consider the process unresponsive.

        The MCU sketch runs its own independent loop() at ~500 Hz (2ms yield),
        completely decoupled from this MPU-side sleep cycle.
        """
        # Sleep for 5 seconds to reduce MPU CPU core consumption while the
        # application is active. The MCU sketch continues sampling sensors
        # and printing events independently during this sleep.
        time.sleep(5)


# Guard clause to ensure the script runs as the main entry point rather than
# an imported module. This is the standard Python idiom for executable scripts.
if __name__ == "__main__":
    # Instantiate the production application object.
    app_instance = GrinFootballApp()

    # Execute the App Lab framework run routine, registering our class method
    # as the user loop callback. App.run() enters an infinite loop that:
    #   1. Calls user_loop() repeatedly.
    #   2. Handles SIGTERM/SIGINT for graceful shutdown.
    #   3. Manages the MCU sketch lifecycle (start/stop).
    App.run(user_loop=app_instance.run_loop)
