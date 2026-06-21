"""
sensor_manager.py — IR Sensor Data Acquisition & Processing Module
===================================================================

This module handles all communication with the MCU (STM32U585) to read the
15 IR proximity sensors on the football pitch, and processes the raw bitmask
data into meaningful ball state information (position, trajectory, speed).

Technical Architecture:
    - Communicates with the MCU via Bridge RPC (arduino-router service)
    - The MCU packs 15 sensor readings into a uint16_t bitmask
    - This module unpacks the bitmask into a 3×5 boolean grid
    - Derives ball position (column), direction, and shot lifecycle events

Data Flow:
    MPU (this module)
        → Bridge.call("get_sensors")
        → MCU reads 15 IR digital pins
        → MCU returns packed uint16_t bitmask
        → This module unpacks into 3×5 grid
        → Derives ball state (position, trajectory, events)

Dependencies:
    - arduino.app_utils.Bridge: RPC communication with the MCU
    - config: Hardware constants and timing parameters
"""

# Standard library imports for time tracking and type annotations
import time  # Used for timestamping detections and calculating intervals

# Project-level configuration constants
from config import (
    SENSOR_COLS,          # Number of sensors per row (5)
    SENSOR_ROWS,          # Number of sensor rows (3)
    ROW_FRONT,            # Index 0 — front row (first ball detection)
    ROW_MIDDLE,           # Index 1 — middle row (approach detection)
    ROW_GOAL,             # Index 2 — goal line row (goal detection)
    BITMASK_ROW_OFFSETS,  # Bit offset for each row in the packed uint16_t
    BITMASK_COL_MASK,     # 5-bit mask (0x1F) to extract one row's data
    NO_DETECTION,         # Sentinel value (-1) for "no ball in this row"
    DEBOUNCE_MS,          # Minimum activation time to count as valid detection
)


class SensorManager:
    """
    Manages IR sensor data acquisition from the MCU and processes raw
    bitmask readings into structured ball state information.

    The sensor grid is a 3×5 matrix of IR proximity sensors:
        - Row 0 (Front):  Detects the ball entering the pitch
        - Row 1 (Middle): Detects the ball approaching the goalkeeper
        - Row 2 (Goal):   Detects if the ball passed the goalkeeper (goal scored)

    Each sensor outputs HIGH (no ball) or LOW (ball detected) based on
    reflected infrared light. The MCU reads all 15 sensors and packs them
    into a single uint16_t for efficient RPC transfer.

    Attributes:
        bridge: Reference to the Bridge RPC module for MCU communication
        grid: 3×5 list of booleans — current sensor state (True = ball detected)
        prev_grid: Previous frame's grid — used for motion/trajectory detection
        last_poll_time: Timestamp (ms) of the most recent sensor poll
        front_col: Column where ball was last detected in front row (-1 if none)
        middle_col: Column where ball was last detected in middle row (-1 if none)
        goal_col: Column where ball was last detected in goal row (-1 if none)
        shot_active: Boolean flag — True if a shot is currently in progress
        shot_start_time: Timestamp (ms) when the current shot was first detected
    """

    def __init__(self, bridge):
        """
        Initialise the SensorManager with a reference to the Bridge RPC module.

        Args:
            bridge: The arduino.app_utils.Bridge module, used to call MCU
                    functions like get_sensors() over the RPC channel.
        """
        # Store reference to the Bridge RPC module for calling MCU functions.
        # The Bridge communicates via the arduino-router Unix socket at
        # /var/run/arduino-router.sock using MessagePack serialisation.
        self.bridge = bridge

        # Current sensor grid state: 3 rows × 5 columns of booleans.
        # True means ball detected at that position, False means clear.
        # Initialised to all-clear (no ball on pitch).
        self.grid = [[False] * SENSOR_COLS for _ in range(SENSOR_ROWS)]

        # Previous frame's grid state — kept to detect motion between polls.
        # By comparing grid vs prev_grid, we can determine ball direction.
        self.prev_grid = [[False] * SENSOR_COLS for _ in range(SENSOR_ROWS)]

        # Timestamp of the last sensor poll in milliseconds (monotonic clock).
        # Used to enforce polling intervals and calculate time deltas.
        self.last_poll_time = 0

        # Per-row ball column detection results (0-4 or -1 for no detection).
        # These represent the most recent detected ball column in each row.
        self.front_col = NO_DETECTION   # Ball column in front row
        self.middle_col = NO_DETECTION  # Ball column in middle row
        self.goal_col = NO_DETECTION    # Ball column in goal row

        # Shot lifecycle tracking.
        # A "shot" begins when the ball is first detected in the front row
        # and ends when the ball leaves all sensors or a goal is scored.
        self.shot_active = False        # True while a shot is in progress
        self.shot_start_time = 0        # Timestamp when current shot began

    def poll_sensors(self):
        """
        Fetch the latest sensor data from the MCU via Bridge RPC and update
        the internal grid state.

        This method calls the MCU's 'get_sensors' RPC function, which returns
        a packed uint16_t bitmask containing all 15 sensor readings. The
        bitmask is then unpacked into the 3×5 boolean grid.

        RPC Call Details:
            Function: "get_sensors" (registered on MCU via Bridge.provide_safe)
            Returns: uint16_t — 15-bit packed sensor data
            Latency: Typically <5ms over the arduino-router serial bridge

        Returns:
            bool: True if the poll succeeded and data was updated,
                  False if the RPC call failed (e.g., Bridge disconnected).
        """
        try:
            # Call the MCU's get_sensors function via Bridge RPC.
            # This triggers a MessagePack RPC request through the arduino-router
            # service, which forwards it to the MCU over the /dev/ttyHS1 serial
            # interface. The MCU responds with the packed sensor bitmask.
            raw_bitmask = self.bridge.call("get_sensors")

            # Record the poll timestamp using monotonic time to avoid
            # issues with system clock adjustments (NTP jumps, etc.).
            self.last_poll_time = int(time.monotonic() * 1000)

            # Preserve the current grid as the previous frame before updating.
            # This enables motion detection by comparing consecutive frames.
            self.prev_grid = [row[:] for row in self.grid]

            # Unpack the raw bitmask into the 3×5 boolean grid.
            self._unpack_bitmask(raw_bitmask)

            # Update per-row ball column detection results.
            self._update_column_detections()

            # Update shot lifecycle state (active/inactive, timing).
            self._update_shot_state()

            return True

        except Exception as e:
            # RPC call failed — log the error and return False.
            # Common failure causes:
            #   - arduino-router service not running
            #   - MCU not responding (firmware crash or reset)
            #   - Bridge not initialised (Bridge.begin() not called on MCU)
            print(f"[SensorManager] RPC poll failed: {e}")
            return False

    def _unpack_bitmask(self, bitmask):
        """
        Unpack a uint16_t bitmask into the 3×5 boolean sensor grid.

        The bitmask encoding (LSB → MSB):
            Bits  0-4:  Row 0 (Front)  — sensors S0–S4
            Bits  5-9:  Row 1 (Middle) — sensors S5–S9
            Bits 10-14: Row 2 (Goal)   — sensors S10–S14

        Args:
            bitmask (int): The packed 15-bit sensor data from the MCU.
                           Bit value 1 = ball detected, 0 = no ball.
        """
        for row_idx in range(SENSOR_ROWS):
            # Right-shift the bitmask by the row's bit offset to align
            # the target row's bits to the LSB position, then mask off
            # the lower 5 bits to isolate just this row's sensor values.
            row_bits = (bitmask >> BITMASK_ROW_OFFSETS[row_idx]) & BITMASK_COL_MASK

            for col_idx in range(SENSOR_COLS):
                # Check if the bit at position col_idx is set (1 = ball detected).
                # The bitwise AND with (1 << col_idx) isolates the specific column's bit.
                self.grid[row_idx][col_idx] = bool(row_bits & (1 << col_idx))

    def _update_column_detections(self):
        """
        Scan each row of the sensor grid and determine which column (0-4)
        the ball is detected in. If multiple columns detect the ball
        simultaneously (e.g., ball spanning two sensors), the centre-most
        detection is chosen via weighted average.

        Updates:
            self.front_col:  Ball column in front row (or NO_DETECTION)
            self.middle_col: Ball column in middle row (or NO_DETECTION)
            self.goal_col:   Ball column in goal row (or NO_DETECTION)
        """
        # Process each row independently to find the ball's column position.
        self.front_col = self._find_ball_column(ROW_FRONT)
        self.middle_col = self._find_ball_column(ROW_MIDDLE)
        self.goal_col = self._find_ball_column(ROW_GOAL)

    def _find_ball_column(self, row_idx):
        """
        Find the column where the ball is detected in a given sensor row.

        If multiple sensors in the row detect the ball (ball spanning two
        adjacent sensors), the arithmetic mean of their indices is computed
        and rounded to the nearest integer. This provides sub-column
        interpolation for more precise ball tracking.

        Args:
            row_idx (int): The row index to scan (0=front, 1=middle, 2=goal).

        Returns:
            int: The detected column index (0-4), or NO_DETECTION (-1) if
                 no ball is detected in this row.
        """
        # Collect indices of all active sensors in this row.
        active_cols = [
            col for col in range(SENSOR_COLS)
            if self.grid[row_idx][col]
        ]

        if not active_cols:
            # No sensors active in this row — ball not present.
            return NO_DETECTION

        # Compute the centroid (weighted average) of active sensor positions.
        # For a single sensor, this returns its index directly.
        # For two adjacent sensors (e.g., [2, 3]), this returns 2.5 → rounds to 2 or 3.
        centroid = sum(active_cols) / len(active_cols)
        return round(centroid)

    def _update_shot_state(self):
        """
        Update the shot lifecycle state based on current sensor readings.

        A shot is considered "active" when the ball is first detected in the
        front row. It remains active until all sensors are clear (ball left
        the pitch or was blocked) or a goal is scored (ball in goal row).

        Updates:
            self.shot_active: Whether a shot is currently in progress
            self.shot_start_time: When the current shot started
        """
        # Check if any sensor on the pitch currently detects the ball.
        any_detection = any(
            self.grid[row][col]
            for row in range(SENSOR_ROWS)
            for col in range(SENSOR_COLS)
        )

        if not self.shot_active and self.front_col != NO_DETECTION:
            # Shot just started — ball entered the front detection zone.
            self.shot_active = True
            self.shot_start_time = int(time.monotonic() * 1000)

        elif self.shot_active and not any_detection:
            # Shot ended — ball left all sensor zones (blocked, missed, or removed).
            self.shot_active = False

    def is_goal_scored(self):
        """
        Check if a goal has been scored in the current frame.

        A goal is scored when the ball is detected in the goal row (Row 2),
        which is positioned behind the goalkeeper. This means the ball
        successfully passed the goalkeeper's defensive position.

        Returns:
            bool: True if ball detected in goal row, False otherwise.
        """
        return self.goal_col != NO_DETECTION

    def get_goal_column(self):
        """
        Get the column where the goal was scored.

        This information is used by the RL agent to understand WHERE the
        goal was scored, enabling it to learn column-specific defensive
        strategies rather than just a binary "goal or no goal" signal.

        Returns:
            int: Column index (0-4) where the ball crossed the goal line,
                 or NO_DETECTION (-1) if no goal detected.
        """
        return self.goal_col

    def get_ball_state(self):
        """
        Get the current ball state as a tuple suitable for Q-learning.

        The state is encoded as (front_col, middle_col) where each value
        is the column (0-4) where the ball was detected, or NO_DETECTION (-1)
        if no ball in that row. This gives a compact representation of the
        ball's position and approach trajectory.

        Returns:
            tuple: (front_col, middle_col) — the Q-learning state vector.
                   Each element is in range [-1, 4] where -1 = no detection.
        """
        return (self.front_col, self.middle_col)

    def is_ball_approaching(self):
        """
        Determine if the ball is actively approaching the goalkeeper.

        The ball is considered "approaching" if it has been detected in
        the middle row (closer to the goalkeeper), indicating the keeper
        needs to react soon.

        Returns:
            bool: True if ball detected in middle row, False otherwise.
        """
        return self.middle_col != NO_DETECTION

    def get_ball_trajectory(self):
        """
        Estimate the ball's lateral trajectory by comparing its column
        position between the front and middle rows.

        Trajectory values:
            - Negative: Ball moving left (towards column 0)
            - Zero: Ball moving straight
            - Positive: Ball moving right (towards column 4)
            - None: Insufficient data (ball not in both rows)

        This is a simple first-order estimate. The RL agent uses the raw
        state (front_col, middle_col) which implicitly encodes trajectory.

        Returns:
            int or None: Column displacement (middle_col - front_col),
                         or None if trajectory cannot be determined.
        """
        if self.front_col == NO_DETECTION or self.middle_col == NO_DETECTION:
            return None

        # Positive displacement means ball moved right between rows.
        # Negative means ball moved left. Zero means straight shot.
        return self.middle_col - self.front_col

    def get_grid_snapshot(self):
        """
        Return a deep copy of the current sensor grid for external use.

        Useful for debugging, logging, or displaying the current pitch state.

        Returns:
            list: A 3×5 list of booleans (deep copy to prevent aliasing).
        """
        return [row[:] for row in self.grid]

    def __repr__(self):
        """
        String representation of the current sensor state for debugging.

        Renders the 3×5 grid as a visual ASCII map where '●' represents
        a detected ball and '○' represents a clear sensor.

        Returns:
            str: Multi-line ASCII grid representation.
        """
        lines = ["[SensorManager Grid]"]
        row_labels = ["FRONT ", "MIDDLE", "GOAL  "]

        for row_idx in range(SENSOR_ROWS):
            # Map each sensor to a filled or empty circle character.
            cells = ["●" if self.grid[row_idx][col] else "○"
                     for col in range(SENSOR_COLS)]
            lines.append(f"  {row_labels[row_idx]}: {' '.join(cells)}")

        lines.append(f"  State: front={self.front_col} mid={self.middle_col} goal={self.goal_col}")
        lines.append(f"  Shot Active: {self.shot_active}")
        return "\n".join(lines)
