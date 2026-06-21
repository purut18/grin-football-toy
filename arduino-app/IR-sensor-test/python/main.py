"""
main.py — IR Tracker Sensor Test (MPU Orchestrator App)
======================================================

This script runs on the MPU (Qualcomm QRB2210 microprocessor) of the Arduino Uno Q.
It initializes the App Lab framework runtime environment and enters an idle loop,
allowing the MCU sketch (running on the STM32U585) to perform sensor reading and
console printing independently.

Technical Specifications:
-------------------------
1. MPU-MCU Interface:
   - Communication channel is governed by the arduino-router service daemon.
   - The MPU initiates and maintains the application daemon process lifecycle.
   - The MCU sketch uses the Bridge RPC library to print to the standard output,
     which is redirected to the MPU and presented in the App Lab CLI or console.
2. Architecture:
   - Follows the Arduino App Lab framework convention.
   - Implements a class-based application manager pattern (IRTestApp) to hold
     application state and run loops.
"""

# Import the App class from arduino.app_utils which provides the runtime loop and signals handling.
from arduino.app_utils import App
# Import the time module to provide standard delay/sleep functions.
import time

class IRTestApp:
    """
    IRTestApp
    Orchestrates the lifecycle of the MPU-side diagnostic test application.
    Maintains an idle state to allow the MCU sketch to execute and output serial data.
    """
    def __init__(self):
        # Initialise configuration parameters. In this diagnostic app, no state is needed.
        pass

    def run_loop(self):
        """
        run_loop
        The main execution cycle called by the App framework.
        Suspends MPU execution to conserve processor cycles since the test is driven by the MCU.
        """
        # Sleep for 5 seconds to reduce MPU CPU core consumption while the application is active.
        time.sleep(5)

# Guard clause to ensure the script runs as the main entry point rather than an imported module.
if __name__ == "__main__":
    # Instantiate the test application object.
    app_instance = IRTestApp()
    # Execute the App Lab framework run routine, registering our class method as the user loop.
    App.run(user_loop=app_instance.run_loop)
