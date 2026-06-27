"""
goalkeeper_ai.py — Q-Learning Reinforcement Learning Agent for Goalkeeper
=========================================================================

This module implements the goalkeeper's AI brain using tabular Q-learning,
a model-free off-policy temporal difference (TD) reinforcement learning
algorithm. The agent learns to position the goalkeeper optimally to block
incoming shots by observing ball trajectories and receiving reward signals.

Q-Learning Algorithm Overview:
    Q-learning maintains a table Q(s, a) mapping every (state, action) pair
    to an estimated value. The value represents the expected cumulative
    future reward from taking action 'a' in state 's' and then following
    the optimal policy. After each experience (s, a, r, s'), the table is
    updated using the Bellman equation:

        Q(s, a) ← Q(s, a) + α · [r + γ · max_a'(Q(s', a')) − Q(s, a)]

    Where:
        s  = current state (ball position in front and middle rows)
        a  = action taken (goalkeeper position 0-4)
        r  = reward received (+1 save, -1 goal)
        s' = next state (after the shot resolves)
        α  = learning rate (how fast to update)
        γ  = discount factor (how much to value future rewards)

    Action selection uses ε-greedy policy:
        - With probability ε: choose random action (exploration)
        - With probability 1-ε: choose action with highest Q-value (exploitation)
        - ε decays over time as the agent gains experience

State Space:
    State = (front_col, middle_col) where each is in {-1, 0, 1, 2, 3, 4}
    -1 means no ball detected in that row.
    Total: 6 × 6 = 36 states

Action Space:
    Action = goalkeeper position ∈ {0, 1, 2, 3, 4}
    Maps to 5 discrete lateral positions across the goal width.
    Total: 5 actions

Q-Table Size:
    36 states × 5 actions = 180 entries (trivially fits in memory)

Persistence:
    The Q-table is serialised to JSON and saved to the UNO Q's eMMC storage
    so learned behaviour persists across power cycles.
"""

# Standard library imports
import json      # JSON serialisation for Q-table persistence to filesystem
import os        # Filesystem operations for checking/creating Q-table save path
import random    # Random number generation for ε-greedy exploration policy

# Project-level configuration constants
from config import (
    SENSOR_COLS,         # Number of columns (5) — defines action space size
    KEEPER_POSITIONS,    # Number of keeper positions (5) — same as SENSOR_COLS
    KEEPER_CENTER,       # Centre position index (2) — default starting position
    NO_DETECTION,        # Sentinel value (-1) — no ball detected in row
    LEARNING_RATE,       # α — Step size for Q-value updates (0.3)
    DISCOUNT_FACTOR,     # γ — Future reward discount factor (0.9)
    EXPLORATION_RATE,    # ε — Initial exploration probability (1.0)
    EXPLORATION_MIN,     # ε_min — Minimum exploration rate floor (0.05)
    EXPLORATION_DECAY,   # ε_decay — Decay multiplier per episode (0.995)
    REWARD_SAVE,         # Positive reward for blocking a shot (+1.0)
    REWARD_GOAL,         # Negative reward for conceding a goal (-1.0)
    REWARD_IDLE,         # Neutral reward for idle timesteps (0.0)
    Q_TABLE_SAVE_PATH,   # Filesystem path for Q-table JSON persistence
    Q_TABLE_SAVE_INTERVAL,  # Episodes between Q-table saves to eMMC (10)
    TOTAL_STATES,        # Total number of unique states (36)
    TOTAL_ACTIONS,       # Total number of possible actions (5)
)


class GoalkeeperAI:
    """
    Tabular Q-learning agent that controls the goalkeeper's lateral position.

    The agent observes the ball's position (which column it's in across the
    front and middle sensor rows), selects a goalkeeper position (0-4), and
    learns from the outcome (save or goal) to improve its positioning over
    time. Initially the agent is completely random (ε=1.0), but as it
    accumulates experience, it gradually shifts to exploiting learned knowledge.

    Attributes:
        q_table (dict): Maps state-action string keys to Q-values (floats).
                        Key format: "(front_col,middle_col)_action"
        epsilon (float): Current exploration rate (decays over time).
        episode_count (int): Total number of completed episodes (shots).
        current_state (tuple): The state when the agent last chose an action.
        current_action (int): The most recently chosen action (keeper position).
        total_saves (int): Lifetime count of successful saves.
        total_goals (int): Lifetime count of goals conceded.
    """

    def __init__(self):
        """
        Initialise the Q-learning agent with an empty Q-table and
        attempt to load a previously saved Q-table from disk.
        """
        # The Q-table is a dictionary mapping "(state)_action" string keys
        # to float Q-values. Using a dictionary (rather than a 2D array)
        # allows sparse storage and easy JSON serialisation.
        # Uninitialised state-action pairs default to 0.0 (optimistic start
        # would use positive values, but zero-init is simpler and works
        # well for small state spaces).
        self.q_table = {}

        # Current exploration rate — starts at EXPLORATION_RATE (1.0 = 100% random)
        # and decays by EXPLORATION_DECAY after each episode until it reaches
        # EXPLORATION_MIN. This implements the explore-exploit tradeoff.
        self.epsilon = EXPLORATION_RATE

        # Episode counter — tracks total number of completed shots.
        # Used to determine when to save the Q-table and for logging.
        self.episode_count = 0

        # Current episode state — tracks the state and action of the
        # ongoing shot so we can update Q-values when the outcome is known.
        self.current_state = None   # (front_col, middle_col) tuple
        self.current_action = None  # int 0-4 (keeper position)

        # Lifetime performance statistics — persisted with the Q-table.
        self.total_saves = 0
        self.total_goals = 0

        # Attempt to load a previously saved Q-table from the filesystem.
        # If found, the agent resumes training from where it left off.
        self._load_q_table()

    def _get_q_key(self, state, action):
        """
        Generate a string key for the Q-table dictionary from a state-action pair.

        The key format is "(front_col,middle_col)_action", e.g., "(2,3)_1".
        Using strings as keys enables direct JSON serialisation/deserialisation
        since JSON object keys must be strings.

        Args:
            state (tuple): The state as (front_col, middle_col).
            action (int): The action (goalkeeper position 0-4).

        Returns:
            str: The Q-table dictionary key.
        """
        return f"{state}_{action}"

    def _get_q_value(self, state, action):
        """
        Look up the Q-value for a given state-action pair.

        If the pair has never been visited, returns 0.0 (default initialisation).
        This is equivalent to assuming zero expected reward for unexplored
        state-action pairs, which is a neutral assumption.

        Args:
            state (tuple): The state as (front_col, middle_col).
            action (int): The action (goalkeeper position 0-4).

        Returns:
            float: The current Q-value estimate for this state-action pair.
        """
        key = self._get_q_key(state, action)
        return self.q_table.get(key, 0.0)

    def _set_q_value(self, state, action, value):
        """
        Store a Q-value for a given state-action pair.

        Args:
            state (tuple): The state as (front_col, middle_col).
            action (int): The action (goalkeeper position 0-4).
            value (float): The new Q-value to store.
        """
        key = self._get_q_key(state, action)
        self.q_table[key] = value

    def choose_action(self, state):
        """
        Select a goalkeeper position using the ε-greedy policy.

        With probability ε (epsilon), a random action is chosen to explore
        unvisited state-action pairs. With probability (1 - ε), the action
        with the highest Q-value for the given state is chosen to exploit
        learned knowledge.

        If multiple actions have the same maximum Q-value, one is chosen
        randomly to break ties and promote exploration.

        Args:
            state (tuple): The current ball state as (front_col, middle_col).

        Returns:
            int: The chosen goalkeeper position (0-4).
        """
        # Store the current state and action for later Q-value update
        # when the shot outcome becomes known.
        self.current_state = state

        if random.random() < self.epsilon:
            # EXPLORATION: Choose a random action.
            # This ensures the agent tries all positions, even ones it
            # currently thinks are suboptimal. Essential for discovering
            # better strategies it hasn't tried yet.
            action = random.randint(0, KEEPER_POSITIONS - 1)
        else:
            # EXPLOITATION: Choose the action with the highest Q-value.
            # This is the "greedy" part — the agent uses its accumulated
            # knowledge to make the best decision it can.
            action = self._get_best_action(state)

        # Record the chosen action for the Q-value update.
        self.current_action = action
        return action

    def _get_best_action(self, state):
        """
        Find the action with the highest Q-value for a given state.

        If multiple actions share the maximum Q-value (common early in
        training when many Q-values are still 0.0), one is selected
        randomly to promote exploration and avoid bias towards lower-indexed
        actions.

        Args:
            state (tuple): The state to evaluate.

        Returns:
            int: The action (0-4) with the highest Q-value.
        """
        # Compute Q-values for all possible actions in this state.
        q_values = [
            self._get_q_value(state, action)
            for action in range(KEEPER_POSITIONS)
        ]

        # Find the maximum Q-value.
        max_q = max(q_values)

        # Collect all actions that achieve this maximum (for tie-breaking).
        best_actions = [
            action for action, q_val in enumerate(q_values)
            if q_val == max_q
        ]

        # Randomly select among tied actions to avoid systematic bias.
        return random.choice(best_actions)

    def update(self, reward, next_state):
        """
        Update the Q-table using the Q-learning update rule (Bellman equation).

        This is called after each shot resolves (save or goal) with the
        reward signal and the terminal state. Since each shot is a single-step
        episode (one action per shot), the next_state is terminal and
        max_a'(Q(s', a')) = 0 for terminal states.

        Q-learning update rule:
            Q(s, a) ← Q(s, a) + α · [r + γ · max_a'(Q(s', a')) − Q(s, a)]

        For terminal states (end of shot):
            Q(s, a) ← Q(s, a) + α · [r − Q(s, a)]
            (since γ · max_a'(Q(s', a')) = 0 when s' is terminal)

        Args:
            reward (float): The reward signal (+1.0 for save, -1.0 for goal).
            next_state (tuple): The state after the shot (typically terminal).
        """
        if self.current_state is None or self.current_action is None:
            # No action was taken — nothing to update.
            # This can happen if update() is called before choose_action().
            return

        # Look up the current Q-value for the (state, action) pair.
        current_q = self._get_q_value(self.current_state, self.current_action)

        # Compute the maximum Q-value achievable from the next state.
        # For terminal states (shot resolved), this represents what the
        # agent expects to get if it were to face this state again.
        max_next_q = max(
            self._get_q_value(next_state, a)
            for a in range(KEEPER_POSITIONS)
        )

        # Apply the Q-learning update rule (Bellman equation).
        # The TD error (temporal difference error) measures how far off
        # our current estimate was from the observed reward + best future value.
        td_target = reward + DISCOUNT_FACTOR * max_next_q  # What we observed
        td_error = td_target - current_q                    # How wrong we were
        new_q = current_q + LEARNING_RATE * td_error        # Corrected estimate

        # Store the updated Q-value.
        self._set_q_value(self.current_state, self.current_action, new_q)

        # Update performance statistics.
        if reward > 0:
            self.total_saves += 1
        elif reward < 0:
            self.total_goals += 1

        # Increment episode counter and decay exploration rate.
        self.episode_count += 1
        self._decay_epsilon()

        # Periodically save the Q-table to persist learned knowledge.
        if self.episode_count % Q_TABLE_SAVE_INTERVAL == 0:
            self._save_q_table()

        # Clear current episode tracking for the next shot.
        self.current_state = None
        self.current_action = None

    def _decay_epsilon(self):
        """
        Decay the exploration rate (ε) after each episode.

        Uses multiplicative decay: ε ← ε × ε_decay, clamped to ε_min.
        This provides exponential decay from 1.0 towards the minimum,
        gradually shifting the agent from exploration (random actions) to
        exploitation (learned optimal actions).

        Decay trajectory (approximate):
            Episode   0:   ε = 1.000 (100% random)
            Episode 100:   ε = 0.606 (60% random)
            Episode 300:   ε = 0.223 (22% random)
            Episode 500:   ε = 0.082 (8% random)
            Episode 600+:  ε = 0.050 (5% random — floor reached)
        """
        self.epsilon = max(
            EXPLORATION_MIN,
            self.epsilon * EXPLORATION_DECAY
        )

    def _save_q_table(self):
        """
        Serialise the Q-table, metadata, and statistics to a JSON file
        on the UNO Q's eMMC storage for persistence across power cycles.

        The save file contains:
            - q_table: The full state-action → Q-value mapping
            - epsilon: Current exploration rate (resume decay correctly)
            - episode_count: Total episodes completed
            - total_saves: Lifetime save count
            - total_goals: Lifetime goal count

        Writes are batched (every Q_TABLE_SAVE_INTERVAL episodes) to reduce
        eMMC write wear. The eMMC has limited write endurance (~3000-10000
        P/E cycles per block), so frequent small writes should be avoided.
        """
        try:
            # Ensure the parent directory exists on the filesystem.
            save_dir = os.path.dirname(Q_TABLE_SAVE_PATH)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)

            # Bundle the Q-table with metadata for complete state restoration.
            save_data = {
                "q_table": self.q_table,
                "epsilon": self.epsilon,
                "episode_count": self.episode_count,
                "total_saves": self.total_saves,
                "total_goals": self.total_goals,
            }

            # Write atomically by using a temporary file and renaming.
            # This prevents data corruption if the board loses power mid-write.
            temp_path = Q_TABLE_SAVE_PATH + ".tmp"
            with open(temp_path, "w") as f:
                json.dump(save_data, f, indent=2)

            # Atomic rename — on Linux/ext4, rename() is atomic within
            # the same filesystem, guaranteeing the file is either fully
            # the old version or fully the new version, never a partial write.
            os.rename(temp_path, Q_TABLE_SAVE_PATH)

            print(f"[GoalkeeperAI] Q-table saved (episode {self.episode_count}, "
                  f"ε={self.epsilon:.3f}, saves={self.total_saves}, "
                  f"goals={self.total_goals})")

        except Exception as e:
            print(f"[GoalkeeperAI] Failed to save Q-table: {e}")

    def _load_q_table(self):
        """
        Load a previously saved Q-table and metadata from the filesystem.

        If the file exists and is valid JSON, the agent resumes training
        from the saved state. If the file doesn't exist or is corrupted,
        the agent starts fresh with an empty Q-table and full exploration.
        """
        try:
            if not os.path.exists(Q_TABLE_SAVE_PATH):
                print("[GoalkeeperAI] No saved Q-table found — starting fresh.")
                return

            with open(Q_TABLE_SAVE_PATH, "r") as f:
                save_data = json.load(f)

            # Restore all saved state.
            self.q_table = save_data.get("q_table", {})
            self.epsilon = save_data.get("epsilon", EXPLORATION_RATE)
            self.episode_count = save_data.get("episode_count", 0)
            self.total_saves = save_data.get("total_saves", 0)
            self.total_goals = save_data.get("total_goals", 0)

            print(f"[GoalkeeperAI] Loaded Q-table from disk "
                  f"(episode {self.episode_count}, ε={self.epsilon:.3f}, "
                  f"{len(self.q_table)} entries, "
                  f"saves={self.total_saves}, goals={self.total_goals})")

        except (json.JSONDecodeError, KeyError, TypeError) as e:
            print(f"[GoalkeeperAI] Corrupted Q-table file, starting fresh: {e}")
            self.q_table = {}
            self.epsilon = EXPLORATION_RATE
            self.episode_count = 0

    def get_stats(self):
        """
        Return a dictionary of the agent's current performance statistics.

        Useful for logging, debugging, or sending to the mobile app for display.

        Returns:
            dict: Performance and configuration statistics.
        """
        total_shots = self.total_saves + self.total_goals
        save_rate = (self.total_saves / total_shots * 100) if total_shots > 0 else 0.0

        return {
            "episode_count": self.episode_count,
            "epsilon": round(self.epsilon, 4),
            "total_saves": self.total_saves,
            "total_goals": self.total_goals,
            "save_rate_pct": round(save_rate, 1),
            "q_table_size": len(self.q_table),
        }

    def force_save(self):
        """
        Force an immediate Q-table save to disk, regardless of the
        save interval. Call this during shutdown or before power-off.
        """
        self._save_q_table()

    def __repr__(self):
        """
        String representation for debugging.

        Returns:
            str: Agent status summary.
        """
        stats = self.get_stats()
        return (
            f"[GoalkeeperAI] Episode: {stats['episode_count']} | "
            f"ε: {stats['epsilon']} | "
            f"Saves: {stats['total_saves']} | "
            f"Goals: {stats['total_goals']} | "
            f"Save Rate: {stats['save_rate_pct']}%"
        )
