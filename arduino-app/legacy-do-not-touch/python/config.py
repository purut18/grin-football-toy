"""
config.py — Central Configuration Module for GRIN Football Toy
==============================================================

This file defines all hardware constants, pin mappings, reinforcement learning
hyperparameters, and system-wide configuration values used across the entire
application. By centralising these values, we ensure consistency between the
sensor manager, the goalkeeper AI, and the main orchestration loop.

Technical Specifications:
    - Hardware Target: Arduino UNO Q (Qualcomm QRB2210 MPU + STM32U585 MCU)
    - Sensor Grid: 3 rows × 5 columns of IR proximity sensors
    - Actuator: Micro servo motor controlling goalkeeper lateral movement
    - Communication: Bridge RPC (MessagePack over Unix socket via arduino-router)
"""

# =============================================================================
# SENSOR GRID CONFIGURATION
# =============================================================================
# The pitch has 3 rows of IR sensors, each row containing 5 sensors.
# Sensors are laid flat on the pitch surface, facing upward, detecting the
# ball as it passes overhead via reflected infrared light.
#
# Physical Layout (top-down view, ball travels from top to bottom):
#
#   [Shooter End]
#   ┌─────────────────────────────┐
#   │  S0   S1   S2   S3   S4    │  ← ROW 1 (Front) — First detection zone
#   │                             │
#   │  S5   S6   S7   S8   S9    │  ← ROW 2 (Middle) — Approach zone
#   │                             │
#   │       ████ KEEPER ████      │  ← Goalkeeper servo position
#   │                             │
#   │  S10  S11  S12  S13  S14   │  ← ROW 3 (Goal Line) — Goal detection
#   └─────────────────────────────┘
#   [Goal Net]

SENSOR_ROWS = 3           # Total number of sensor rows on the pitch
SENSOR_COLS = 5           # Number of sensors per row (defines positional resolution)
TOTAL_SENSORS = SENSOR_ROWS * SENSOR_COLS  # 15 sensors total

# Row indices — used to extract specific rows from the packed bitmask
# returned by the MCU's get_sensors() RPC call.
ROW_FRONT = 0             # Row closest to the shooter — detects ball entry
ROW_MIDDLE = 1            # Row between front and goalkeeper — detects approach trajectory
ROW_GOAL = 2              # Row behind the goalkeeper — detects goal scoring events

# =============================================================================
# BITMASK ENCODING SCHEME
# =============================================================================
# The MCU packs all 15 sensor readings into a single uint16_t bitmask for
# efficient RPC transmission. This avoids the overhead of sending 15 separate
# values over the Bridge, reducing latency and improving responsiveness.
#
# Bit layout (LSB → MSB):
#   Bits  0-4:  Row 1 (Front)   — sensors S0 through S4
#   Bits  5-9:  Row 2 (Middle)  — sensors S5 through S9
#   Bits 10-14: Row 3 (Goal)    — sensors S10 through S14
#
# A bit value of 1 indicates ball detection (IR reflection detected).
# A bit value of 0 indicates no ball (clear path, no reflection).

BITMASK_ROW_OFFSETS = [0, 5, 10]  # Starting bit position for each row
BITMASK_COL_MASK = 0x1F           # 5-bit mask (0b11111) to extract one row

# =============================================================================
# GOALKEEPER SERVO CONFIGURATION
# =============================================================================
# The goalkeeper slides laterally across 5 discrete positions (matching the
# 5 sensor columns). The servo angle is linearly interpolated between the
# minimum and maximum angles to map these positions to physical movement.

KEEPER_POSITIONS = 5      # Number of discrete positions (matches SENSOR_COLS)
KEEPER_CENTER = 2         # Index of the centre position (0-indexed, middle column)
SERVO_MIN_ANGLE = 0       # Servo angle in degrees for position 0 (far left)
SERVO_MAX_ANGLE = 180     # Servo angle in degrees for position 4 (far right)

# =============================================================================
# REINFORCEMENT LEARNING HYPERPARAMETERS
# =============================================================================
# The goalkeeper AI uses Q-learning, a model-free temporal difference (TD)
# reinforcement learning algorithm. Q-learning learns an action-value function
# Q(s, a) that estimates the expected cumulative reward of taking action 'a'
# in state 's' and then following the optimal policy thereafter.
#
# Update rule:
#   Q(s, a) ← Q(s, a) + α · [r + γ · max_a'(Q(s', a')) − Q(s, a)]
#
# Where:
#   α (LEARNING_RATE)      = step size for value updates
#   γ (DISCOUNT_FACTOR)    = weight of future rewards vs immediate rewards
#   ε (EXPLORATION_RATE)   = probability of choosing a random action (exploration)
#   ε_min (EXPLORATION_MIN)= minimum exploration rate after decay
#   ε_decay (EXPLORATION_DECAY) = multiplicative decay factor per episode

LEARNING_RATE = 0.3         # α — How aggressively to update Q-values per experience.
                            # Higher values mean faster learning but more instability.
                            # Range: (0, 1]. Typical: 0.1–0.5 for small state spaces.

DISCOUNT_FACTOR = 0.9       # γ — How much to value future rewards vs. immediate.
                            # 0 = purely myopic (only current reward matters).
                            # 1 = fully considers future rewards.
                            # 0.9 is a good default for short-horizon tasks.

EXPLORATION_RATE = 1.0      # ε — Initial probability of random action (100% random).
                            # Starts high for maximum exploration, decays over time.
                            # Uses ε-greedy policy: with probability ε, pick random;
                            # otherwise, pick the action with highest Q-value.

EXPLORATION_MIN = 0.05      # ε_min — Floor for exploration rate to prevent the agent
                            # from becoming fully deterministic. Ensures it always
                            # tries new things ~5% of the time to adapt to changes.

EXPLORATION_DECAY = 0.995   # ε_decay — Multiplied to ε after each episode (shot).
                            # Controls the transition from exploration to exploitation.
                            # At 0.995: ~500 episodes to reach ε_min from 1.0.

# =============================================================================
# REWARD SIGNAL VALUES
# =============================================================================
# These scalar values define the reinforcement signal the agent receives after
# each shot. Positive rewards reinforce good goalkeeper positioning, negative
# rewards penalise failures, and small positive rewards encourage any action.

REWARD_SAVE = 1.0           # Reward when goalkeeper successfully blocks the ball.
                            # The ball was detected approaching but NOT in the goal row.

REWARD_GOAL = -1.0          # Penalty when a goal is scored (ball reaches goal row).
                            # The stronger this signal relative to REWARD_SAVE, the
                            # more the agent prioritises avoiding goals over all else.

REWARD_IDLE = 0.0           # Neutral reward for timesteps where nothing happens.
                            # No ball detected, no action consequences.

# =============================================================================
# Q-TABLE PERSISTENCE
# =============================================================================
# The Q-table is saved to the UNO Q's filesystem (eMMC storage) so the
# goalkeeper's learned behaviour persists across power cycles. The agent
# resumes training from where it left off rather than starting from scratch.

Q_TABLE_SAVE_PATH = "/home/user/grin-football/q_table.json"  # Path on UNO Q filesystem
Q_TABLE_SAVE_INTERVAL = 10   # Save Q-table every N episodes to reduce eMMC writes.
                              # eMMC has limited write endurance (~3000-10000 P/E cycles),
                              # so batching writes extends the storage lifespan.

# =============================================================================
# TIMING & POLLING CONFIGURATION
# =============================================================================
# These values control how frequently the MPU polls the MCU for sensor data
# and how long to wait before considering a shot complete.

SENSOR_POLL_INTERVAL_MS = 20    # Milliseconds between sensor polls (50 Hz polling rate).
                                 # Must be fast enough to track ball at play speeds.
                                 # The MCU reads at ~100 Hz; polling at 50 Hz is sufficient.

SHOT_TIMEOUT_MS = 3000          # Maximum duration in milliseconds for a single shot.
                                 # If no new sensor activity after this period, the shot
                                 # is considered over (ball removed or stuck).

DEBOUNCE_MS = 50                # Minimum time a sensor must be active to count as a
                                 # valid detection. Prevents false positives from noise,
                                 # vibrations, or ambient IR interference.

# =============================================================================
# STATE ENCODING
# =============================================================================
# The Q-learning state is encoded as a tuple (row1_col, row2_col) where each
# value represents which column (0-4) the ball was detected in, or -1 if no
# ball was detected in that row. This gives 6 × 6 = 36 possible states.
#
# The total Q-table size is: 36 states × 5 actions = 180 entries.
# This is small enough to store in memory and converges quickly.

NO_DETECTION = -1             # Sentinel value meaning "no ball detected in this row"
STATE_DIMS = (SENSOR_COLS + 1, SENSOR_COLS + 1)  # (6, 6) — possible values per row
TOTAL_STATES = STATE_DIMS[0] * STATE_DIMS[1]     # 36 unique states
TOTAL_ACTIONS = KEEPER_POSITIONS                   # 5 possible goalkeeper positions
