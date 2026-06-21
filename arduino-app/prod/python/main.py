"""
main.py — GRIN Football Toy Application Entry Point (MPU / Qualcomm QRB2210)
==============================================================================

This is the main orchestration script that runs on the Qualcomm QRB2210
microprocessor (MPU) under Debian Linux. It ties together the sensor manager,
the Q-learning goalkeeper AI, and the Bridge RPC communication with the
STM32U585 microcontroller (MCU).

Application Lifecycle:
    1. Initialise the Arduino App framework (App.run handles lifecycle)
    2. Connect to the MCU via Bridge RPC
    3. Poll sensors at ~50 Hz to track ball position
    4. When a shot is detected (ball in front row):
        a. Observe ball state (position in front and middle rows)
        b. Ask the Q-learning agent to choose a goalkeeper position
        c. Send the position command to the MCU (moves servo)
    5. When the shot resolves (ball clears sensors or scores):
        a. Determine outcome (save or goal)
        b. Send reward signal to the Q-learning agent
        c. Agent updates Q-table and decays exploration rate
    6. Repeat forever — the keeper gets smarter with every shot

System Architecture:
    ┌─────────────────────────────────────────────────────┐
    │              MPU (This Script)                       │
    │  ┌──────────────┐  ┌──────────────┐                 │
    │  │ SensorManager│──│ GoalkeeperAI │                 │
    │  │ (poll/parse) │  │ (Q-learning) │                 │
    │  └──────┬───────┘  └──────┬───────┘                 │
    │         │                  │                         │
    │    Bridge.call         Bridge.call                   │
    │   ("get_sensors")   ("set_keeper_pos")              │
    └─────────┬──────────────────┬────────────────────────┘
              │    arduino-router (RPC)    │
    ┌─────────┴──────────────────┴────────────────────────┐
    │              MCU (sketch.ino)                        │
    │    IR Sensors (15x)          Servo Motor             │
    └─────────────────────────────────────────────────────┘

Dependencies:
    - arduino.app_utils: App lifecycle, Bridge RPC, Leds control
    - sensor_manager: IR sensor data acquisition and processing
    - goalkeeper_ai: Q-learning reinforcement learning agent
    - config: Hardware and algorithm configuration constants
"""

# Standard library imports
import time    # Sleep and timing functions for polling intervals
import signal  # Signal handling for graceful shutdown (SIGTERM, SIGINT)
import sys     # System exit for clean shutdown

# Arduino App Lab framework imports.
# These modules are pre-installed on the UNO Q's Debian Linux image and
# provide the application lifecycle management and RPC bridge.
from arduino.app_utils import App      # App lifecycle — manages startup/shutdown
from arduino.app_utils import Bridge   # RPC bridge — communication with MCU
from arduino.app_utils import Leds     # LED control — MPU-side RGB LEDs 1 & 2

# Project module imports
from sensor_manager import SensorManager  # IR sensor polling and state extraction
from goalkeeper_ai import GoalkeeperAI    # Q-learning agent for keeper positioning
from config import (
    SENSOR_POLL_INTERVAL_MS,  # Polling interval in ms (20ms = 50Hz)
    SHOT_TIMEOUT_MS,          # Max shot duration before timeout (3000ms)
    REWARD_SAVE,              # Reward for successful save (+1.0)
    REWARD_GOAL,              # Penalty for conceding a goal (-1.0)
    KEEPER_CENTER,            # Default centre position for goalkeeper (2)
    NO_DETECTION,             # Sentinel for "no ball detected" (-1)
)


# =============================================================================
# APPLICATION STATE MACHINE
# =============================================================================
# The main loop operates as a finite state machine (FSM) with three states:
#
#   IDLE ──[ball detected in front row]──→ TRACKING
#   TRACKING ──[ball in middle row]──→ REACTING
#   TRACKING ──[timeout / ball lost]──→ IDLE
#   REACTING ──[goal scored]──→ RESOLVING
#   REACTING ──[ball cleared]──→ RESOLVING
#   RESOLVING ──[Q-table updated]──→ IDLE

STATE_IDLE = "IDLE"            # Waiting for a shot — no ball on pitch
STATE_TRACKING = "TRACKING"    # Ball detected in front row — tracking trajectory
STATE_REACTING = "REACTING"    # Ball approaching goalkeeper — action taken
STATE_RESOLVING = "RESOLVING"  # Shot resolved — processing outcome


class GrinFootballApp:
    """
    Main application class that orchestrates the GRIN Football Toy.

    Manages the game loop, state machine transitions, sensor polling,
    AI decision-making, and servo control via the Bridge RPC.

    Attributes:
        sensors (SensorManager): Handles IR sensor data from MCU
        ai (GoalkeeperAI): Q-learning agent for goalkeeper positioning
        state (str): Current FSM state (IDLE/TRACKING/REACTING/RESOLVING)
        shot_start_time (float): Timestamp when current shot was first detected
        action_taken (bool): Whether the AI has already chosen a position for this shot
        last_state_observed (tuple): The ball state when the AI made its decision
    """

    def __init__(self):
        """
        Initialise the application components.

        Creates the sensor manager and AI agent instances. The sensor manager
        receives the Bridge module reference for RPC communication with the MCU.
        """
        # Initialise the sensor manager with the Bridge RPC module.
        # This allows it to call get_sensors() on the MCU.
        self.sensors = SensorManager(Bridge)

        # Initialise the Q-learning goalkeeper AI.
        # This loads any previously saved Q-table from disk.
        self.ai = GoalkeeperAI()

        # State machine — starts in IDLE, waiting for a shot.
        self.state = STATE_IDLE

        # Shot timing — tracks when the current shot started.
        self.shot_start_time = 0.0

        # Action tracking — ensures the AI only decides once per shot.
        self.action_taken = False

        # The ball state observed when the AI made its decision.
        # Stored for the Q-table update when the shot resolves.
        self.last_state_observed = (NO_DETECTION, NO_DETECTION)

        # Set goalkeeper to centre position on startup via Bridge RPC.
        # This ensures the servo is in a known good position before play.
        try:
            Bridge.call("set_keeper_pos", KEEPER_CENTER)
            print("[App] Goalkeeper centred at startup.")
        except Exception as e:
            print(f"[App] Warning: Could not centre goalkeeper: {e}")

        # Set LED 1 to green to indicate the app is running and ready.
        # LED 1 is MPU-controlled via /sys/class/leds/ interface.
        try:
            Leds.set_led1_color(0, 1, 0)  # RGB: Green = ready
        except Exception as e:
            print(f"[App] Warning: Could not set status LED: {e}")

        print("[App] GRIN Football Toy initialised.")
        print(f"[App] AI Status: {self.ai}")

    def loop(self):
        """
        Main application loop — called repeatedly by the App framework.

        This function runs the state machine, which manages the lifecycle
        of each shot from detection through resolution. It polls sensors,
        transitions between states, invokes the AI, and sends commands
        to the MCU's servo motor.

        The App.run(user_loop=loop) framework calls this function in a
        tight loop. We control the polling rate internally using sleep().
        """
        # Poll the MCU for the latest sensor data via Bridge RPC.
        if not self.sensors.poll_sensors():
            # RPC failed — wait and retry next iteration.
            time.sleep(0.1)
            return

        # Get the current time for timeout calculations.
        now = time.monotonic()

        # Execute the appropriate state handler based on current FSM state.
        if self.state == STATE_IDLE:
            self._handle_idle()

        elif self.state == STATE_TRACKING:
            self._handle_tracking(now)

        elif self.state == STATE_REACTING:
            self._handle_reacting(now)

        elif self.state == STATE_RESOLVING:
            self._handle_resolving()

        # Sleep to maintain the target polling rate.
        # SENSOR_POLL_INTERVAL_MS = 20ms → 50Hz polling.
        time.sleep(SENSOR_POLL_INTERVAL_MS / 1000.0)

    def _handle_idle(self):
        """
        IDLE state handler — waiting for a new shot.

        Monitors the front sensor row for ball detection. When a ball is
        detected, transitions to TRACKING state and records the start time.
        """
        if self.sensors.front_col != NO_DETECTION:
            # Ball detected in front row — a shot has started!
            self.state = STATE_TRACKING
            self.shot_start_time = time.monotonic()
            self.action_taken = False

            # Flash LED 1 yellow to indicate shot detected.
            try:
                Leds.set_led1_color(1, 1, 0)  # RGB: Yellow = shot in progress
            except Exception:
                pass

            print(f"[App] Shot detected! Ball in column {self.sensors.front_col}")

    def _handle_tracking(self, now):
        """
        TRACKING state handler — ball detected, tracking trajectory.

        Waits for the ball to reach the middle row to determine trajectory,
        then asks the AI to choose a goalkeeper position. Also handles
        timeout if the ball is removed or lost.

        Args:
            now (float): Current monotonic timestamp.
        """
        # Check for timeout — ball may have been removed or stuck.
        elapsed_ms = (now - self.shot_start_time) * 1000
        if elapsed_ms > SHOT_TIMEOUT_MS:
            print("[App] Shot timed out — resetting to IDLE.")
            self.state = STATE_IDLE
            self._reset_keeper()
            return

        # Check if ball has reached the middle row (approaching goalkeeper).
        if self.sensors.is_ball_approaching() and not self.action_taken:
            # Ball is in the middle row — time for the AI to react!
            # Observe the current state (front_col, middle_col).
            ball_state = self.sensors.get_ball_state()
            self.last_state_observed = ball_state

            # Ask the Q-learning agent to choose a goalkeeper position.
            keeper_pos = self.ai.choose_action(ball_state)

            # Send the position command to the MCU's servo via Bridge RPC.
            try:
                Bridge.call("set_keeper_pos", keeper_pos)
                print(f"[App] AI chose position {keeper_pos} "
                      f"for state {ball_state} "
                      f"(ε={self.ai.epsilon:.3f})")
            except Exception as e:
                print(f"[App] Failed to move keeper: {e}")

            self.action_taken = True
            self.state = STATE_REACTING

    def _handle_reacting(self, now):
        """
        REACTING state handler — keeper has moved, waiting for outcome.

        Monitors for two possible outcomes:
            1. Goal scored — ball detected in goal row (behind keeper)
            2. Save — ball disappears from all sensors (blocked or deflected)

        Args:
            now (float): Current monotonic timestamp.
        """
        # Check for timeout.
        elapsed_ms = (now - self.shot_start_time) * 1000
        if elapsed_ms > SHOT_TIMEOUT_MS:
            # Timeout with no goal — assume save (ball removed).
            print("[App] Shot timed out in REACTING — assuming save.")
            self._resolve_shot(is_goal=False)
            return

        # Check if a goal was scored (ball in goal row).
        if self.sensors.is_goal_scored():
            goal_col = self.sensors.get_goal_column()
            print(f"[App] GOAL! Ball scored in column {goal_col}")
            self._resolve_shot(is_goal=True)
            return

        # Check if all sensors are clear (ball blocked or left pitch).
        ball_state = self.sensors.get_ball_state()
        if (ball_state[0] == NO_DETECTION and
                ball_state[1] == NO_DETECTION and
                not self.sensors.is_goal_scored()):
            # No ball detected anywhere — save!
            print("[App] SAVE! Ball blocked.")
            self._resolve_shot(is_goal=False)

    def _resolve_shot(self, is_goal):
        """
        Resolve the current shot — update the AI with the outcome.

        Sends the appropriate reward signal to the Q-learning agent,
        which updates the Q-table and decays the exploration rate.

        Args:
            is_goal (bool): True if a goal was scored, False if saved.
        """
        # Determine the reward signal.
        reward = REWARD_GOAL if is_goal else REWARD_SAVE

        # Get the terminal state (what the sensors show now).
        terminal_state = self.sensors.get_ball_state()

        # Update the Q-learning agent with the experience.
        self.ai.update(reward, terminal_state)

        # Log the outcome and current AI stats.
        stats = self.ai.get_stats()
        outcome_str = "GOAL" if is_goal else "SAVE"
        print(f"[App] Shot resolved: {outcome_str} | "
              f"Episode: {stats['episode_count']} | "
              f"Save Rate: {stats['save_rate_pct']}% | "
              f"ε: {stats['epsilon']}")

        # Update LED colour based on last outcome.
        try:
            if is_goal:
                Leds.set_led1_color(1, 0, 0)  # RGB: Red = goal conceded
            else:
                Leds.set_led1_color(0, 0, 1)  # RGB: Blue = save made
        except Exception:
            pass

        # Transition to RESOLVING state briefly, then back to IDLE.
        self.state = STATE_RESOLVING

    def _handle_resolving(self):
        """
        RESOLVING state handler — brief pause after shot resolution.

        Resets the goalkeeper to centre position and transitions back to
        IDLE to await the next shot. Includes a brief delay for the player
        to retrieve the ball.
        """
        # Wait a moment for the player to retrieve the ball.
        time.sleep(0.5)

        # Reset goalkeeper to centre position.
        self._reset_keeper()

        # Reset LED to green (ready for next shot).
        try:
            Leds.set_led1_color(0, 1, 0)  # RGB: Green = ready
        except Exception:
            pass

        # Return to IDLE state.
        self.state = STATE_IDLE
        print("[App] Ready for next shot.")

    def _reset_keeper(self):
        """
        Move the goalkeeper back to the centre position.

        Called between shots to reset the servo to a neutral starting
        position, giving the keeper equal coverage of all columns.
        """
        try:
            Bridge.call("set_keeper_pos", KEEPER_CENTER)
        except Exception as e:
            print(f"[App] Warning: Could not reset keeper: {e}")


# =============================================================================
# APPLICATION ENTRY POINT
# =============================================================================
# The App.run() function from arduino.app_utils manages the application
# lifecycle. It initialises the RPC bridge, sets up signal handlers, and
# calls user_loop repeatedly until the application is stopped.

# Create the application instance — this initialises sensors, AI, and servo.
app = GrinFootballApp()


def on_shutdown():
    """
    Graceful shutdown handler — saves the Q-table before exit.

    Called when the app is stopped via Arduino App Lab, SIGTERM, or SIGINT.
    Ensures the learned Q-table is persisted to disk so the goalkeeper
    retains its learned behaviour across restarts.
    """
    print("[App] Shutting down — saving Q-table...")
    app.ai.force_save()
    print("[App] Goodbye!")


# Register the shutdown handler for clean exits.
# SIGTERM is sent by systemd when stopping a service.
# SIGINT is sent by Ctrl+C in the terminal.
signal.signal(signal.SIGTERM, lambda sig, frame: (on_shutdown(), sys.exit(0)))
signal.signal(signal.SIGINT, lambda sig, frame: (on_shutdown(), sys.exit(0)))

# Start the application loop via the Arduino App framework.
# App.run() calls app.loop() repeatedly in a managed loop.
App.run(user_loop=app.loop)
