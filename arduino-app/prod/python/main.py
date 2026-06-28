"""
main.py — Production Dual IR Sensor Tracker Orchestrator & Goalkeeper AI (MPU App)
=================================================================================

This script runs natively on the Qualcomm QRB2210 microprocessor (MPU) of the 
Arduino Uno Q under Debian Linux. It acts as the orchestrator for the goalkeeper AI,
subscribing to real-time shot events triggered by the STM32U585 microcontroller (MCU),
making lateral positioning predictions, and running asynchronous training updates.

Technical Specifications:
-------------------------
1. MPU-MCU Interface (Bridge RPC):
   - Communication is handled via MessagePack RPC over a internal serial link (/dev/ttyHS1),
     proxied by the systemd service 'arduino-router'.
   - This script registers two Python handlers:
     - "predict_position" -> Invoked by MCU when Sensor 2 (mid-row) triggers. Runs model inference.
     - "record_outcome"   -> Invoked by MCU when shot resolves. Logs data and runs async training.
   - Using 'Bridge.provide', these methods are exposed to the MCU, allowing low-latency
     synchronous execution.

2. Goalkeeper AI Model:
   - Imported from model.py, encapsulating the ReinforcementModel class.
   - Runs inference synchronously to ensure minimum latency (<5ms) so the keeper servo 
     can move into position before the ball crosses the goal line.
   - Runs self-training asynchronously in a background thread to prevent blocking MCU loop.

3. Process Lifecycle:
   - Uses the 'arduino.app_utils.App' framework to hook system exit signals (SIGTERM/SIGINT)
     and maintain the runtime execution daemon loop.
"""

# Import the App and Bridge modules from the Arduino App Lab platform library.
# App handles system signals (SIGTERM/SIGINT) and monitors process states.
# Bridge handles the serial RPC transport and binding of local handlers.
from arduino.app_utils import App, Bridge

# Import standard sleep delay functions to throttle MPU CPU utilization.
import time

# Import the reinforcement learning agent model and centroid decoding helpers.
from model import ReinforcementModel

# Instantiate the global reinforcement model agent.
# The constructor loads the historical training logs and learned policy weights from disk.
model = ReinforcementModel()


def sync_policy_to_mcu():
    """
    sync_policy_to_mcu
    Synchronizes the MPU reinforcement learning policy weights to the MCU's SRAM policy lookup table.
    Ensures that the MCU can perform zero-latency O(1) predictions in real-time.
    """
    print("[MPU] Synchronizing policy table to MCU...")
    # Safely fetch a copy of the policy mappings from the model under thread lock
    with model.lock:
        policy_copy = dict(model.model_data)

    success_count = 0
    for state_key, action in policy_copy.items():
        try:
            # Parse state key formatted as "s1_s2" (e.g. "12_15")
            s1_str, s2_str = state_key.split("_")
            s1_mask = int(s1_str)
            s2_mask = int(s2_str)
            
            # Send dynamic weight updates to the MCU's policy table via RPC
            Bridge.call("update_policy_state", s1_mask, s2_mask, int(action))
            success_count += 1
        except Exception as e:
            print(f"[MPU] Failed to sync state {state_key} -> {action}: {e}")
            
    print(f"[MPU] Policy synchronization complete. Successfully synced {success_count}/{len(policy_copy)} states.")


def record_outcome(s1_mask, s2_mask, s3_mask, was_goal, keeper_pos):
    """
    record_outcome
    RPC handler invoked by the MCU when a shot is finalized (Sensor 3 triggered or timed out).
    Logs the outcome features and spawns the background self-training routine.
    Once training completes, it automatically triggers policy synchronization to the MCU.
    
    @param s1_mask: int - 5-bit sensor mask for Sensor Row 1
    @param s2_mask: int - 5-bit sensor mask for Sensor Row 2
    @param s3_mask: int - 5-bit sensor mask for Sensor Row 3
    @param was_goal: bool - True if goal scored, False if saved
    @param keeper_pos: int - Goalkeeper column position index (0 to 4)
    @return: bool - True on success, False otherwise
    """
    try:
        # Log outcome and spawn the background thread for model training with MCU sync callback
        model.add_experience(s1_mask, s2_mask, s3_mask, was_goal, keeper_pos, on_train_complete=sync_policy_to_mcu)
        outcome_str = "GOAL" if was_goal else "SAVE"
        print(f"[RPC] record_outcome resolved: {outcome_str} (Keeper={keeper_pos})")
        return True
    except Exception as e:
        print(f"[RPC] Error in record_outcome: {e}")
        return False


class GrinFootballApp:
    """
    GrinFootballApp
    Class-based manager orchestrating the MPU application runtime.
    Attempts to perform the initial policy weight synchronization to the MCU upon start-up,
    retrying every 5 seconds until the MCU's Bridge transport is ready.
    """
    def __init__(self):
        self.synced = False
        print("[App] GRIN Football AI Orchestrator initialized.")

    def run_loop(self):
        """
        run_loop
        Execution loop called repeatedly by App.run() at 5-second intervals.
        Handles the initial boot-up synchronization of the reinforcement learning policy weights.
        """
        if not self.synced:
            try:
                sync_policy_to_mcu()
                self.synced = True
            except Exception as e:
                print(f"[App] MCU Bridge not ready for startup policy sync, retrying... ({e})")
        time.sleep(5)


# Main Entry Point
if __name__ == "__main__":
    # Expose the local python record_outcome handler to the Bridge RPC routing tables.
    # The MCU can then invoke it asynchronously using Bridge.call().
    # predict_position is no longer provided because inference runs locally on the MCU.
    Bridge.provide("record_outcome", record_outcome)
    print("[MPU] Registered RPC method with Bridge: record_outcome")

    # Instantiate the application manager
    app_instance = GrinFootballApp()

    # Execute the App Lab framework mainloop, passing our run_loop callback.
    App.run(user_loop=app_instance.run_loop)
