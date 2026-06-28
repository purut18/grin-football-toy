"""
model.py — Self-Training Reinforcement Goalkeeper AI Model (MPU Component)
==========================================================================

This module implements the goalkeeper's decision-making and self-training logic
running on the MPU (Qualcomm QRB2210 microprocessor) under Debian Linux.
It utilizes a tabular count-based reinforcement policy estimator combined with
a first-order physics trajectory prior to perform zero-shot and adaptive learning.

Technical Specifications:
-------------------------
1. State Space:
   - State = (s1_mask, s2_mask) where each mask is a 5-bit integer (0 to 31)
     representing the accumulated phototransistor activations on Sensor 1 and Sensor 2.
   - Total state space is 32 × 32 = 1024 states, of which a small subset corresponding
     to valid physical trajectories is visited during play.

2. Action Space:
   - Action = Goalkeeper column index ∈ {0, 1, 2, 3, 4} (5 lateral discrete positions).

3. Learning Policy:
   - e-greedy selection (exploration rate epsilon starting at 0.2 and decaying to 0.05).
   - Unexplored states fall back to a physics-based prior prediction:
     c3 = clamp(2 * c2 - c1, 0, 4) where c1 and c2 are centroids of S1 and S2.

4. Asynchronous Training:
   - Updates are triggered in a background daemon thread after each shot resolves.
   - Re-estimates the maximum likelihood action for each state from the logged history:
     policy(state) = argmax_action frequency_counts(state, action)
   - Persistence is managed atomically via JSON serialisation to protect against power failures.
"""

import os        # Filesystem path calculations and atomic file renaming
import json      # JSON serialisation for model parameters and history
import random    # Random choice for exploration and tie-breaking
import threading # Thread locks and background execution for async training

# Define persistence file paths relative to this script directory to ensure portability.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HISTORY_PATH = os.path.join(SCRIPT_DIR, "shot_history.json")
MODEL_PATH = os.path.join(SCRIPT_DIR, "model.json")


def get_active_col(mask):
    """
    get_active_col
    Decodes a 5-bit binary sensor mask to identify the active column index (0-4).
    Uses a centroid calculation (weighted arithmetic mean) if multiple adjacent
    phototransistors are triggered by the ball.
    
    Why: The ball has physical width and can bridge multiple adjacent IR sensors.
         Calculating the centroid provides sub-column spatial accuracy.
    
    How:
      1. Iterate through bits 0 to 4 of the mask.
      2. Record indices where the bit is set.
      3. Compute the arithmetic mean of these indices and round it.
      
    @param mask: int - 5-bit binary sensor mask (0 to 31)
    @return: int - Active column index (0 to 4) or -1 if no sensors are active
    """
    if not mask:
        return -1
        
    active_indices = []
    for i in range(5):
        # Perform bitwise shift and mask to check if the i-th bit is set (1)
        if (mask >> i) & 1:
            active_indices.append(i)
            
    if not active_indices:
        return -1
        
    # Calculate the mean position (centroid) and round to the nearest column index
    centroid = sum(active_indices) / len(active_indices)
    return int(round(centroid))


class ReinforcementModel:
    """
    ReinforcementModel
    A count-based tabular reinforcement learning model that learns the mapping
    from (S1, S2) trigger states to S3 intercept columns.
    
    Uses thread-safe locks to manage read/write operations between the RPC calling
    threads (inference and logging) and the background training thread.
    """
    def __init__(self):
        """
        Constructor
        Initializes the thread safety lock, exploration rate, policy mappings,
        and loads saved data from the filesystem.
        """
        # Reentrant/Mutex lock to serialize access to policy mapping and history variables
        self.lock = threading.Lock()
        
        # Policy mapping dictionary. Key: "s1_s2", Value: optimal action index (0-4)
        self.model_data = {}
        
        # Epsilon-greedy exploration probability. Dictates probability of choosing random position.
        self.epsilon = 0.2
        
        # In-memory database of completed shots
        self.history = []
        
        # Bootstrap the model and history logs from disk
        self.load_model()
        self.load_history()

    def load_model(self):
        """
        load_model
        Deserialises the model weights and exploration rate from model.json.
        """
        with self.lock:
            if os.path.exists(MODEL_PATH):
                try:
                    with open(MODEL_PATH, "r") as f:
                        data = json.load(f)
                        self.model_data = data.get("model", {})
                        self.epsilon = data.get("epsilon", 0.2)
                        print(f"[Model] Loaded model weights with {len(self.model_data)} states (epsilon={self.epsilon:.3f})")
                except Exception as e:
                    print(f"[Model] Error loading model, initializing empty: {e}")
            else:
                print("[Model] No model file found, starting with empty weights.")

    def load_history(self):
        """
        load_history
        Deserialises the historical shot experience database from shot_history.json.
        """
        with self.lock:
            if os.path.exists(HISTORY_PATH):
                try:
                    with open(HISTORY_PATH, "r") as f:
                        self.history = json.load(f)
                        print(f"[Model] Loaded history with {len(self.history)} shots")
                except Exception as e:
                    print(f"[Model] Error loading history, initializing empty: {e}")
            else:
                print("[Model] No history file found, starting fresh.")

    def save_model(self):
        """
        save_model
        Saves the model weights and parameters atomically using a temporary file.
        Why: Atomic renames prevent file corruption during unexpected power cuts.
        """
        try:
            temp_path = MODEL_PATH + ".tmp"
            with open(temp_path, "w") as f:
                json.dump({"model": self.model_data, "epsilon": self.epsilon}, f, indent=2)
            # Perform atomic replace on POSIX systems
            os.rename(temp_path, MODEL_PATH)
        except Exception as e:
            print(f"[Model] Error saving model: {e}")

    def save_history(self):
        """
        save_history
        Saves the logged experience history database atomically to disk.
        """
        try:
            temp_path = HISTORY_PATH + ".tmp"
            with open(temp_path, "w") as f:
                json.dump(self.history, f, indent=2)
            os.rename(temp_path, HISTORY_PATH)
        except Exception as e:
            print(f"[Model] Error saving history: {e}")

    def predict(self, s1_mask, s2_mask):
        """
        predict
        Generates the target goalkeeper position (0-4) using an epsilon-greedy policy.
        
        How:
          1. Generates a random number. If less than epsilon, explores (chooses random action).
          2. Performs a lookup in the learned policy dict.
          3. If the state is unseen, applies the physics extrapolation prior:
             c3 = clamp(2 * c2 - c1, 0, 4)
             
        @param s1_mask: int - 5-bit trigger mask for Sensor Row 1
        @param s2_mask: int - 5-bit trigger mask for Sensor Row 2
        @return: int - Goalkeeper column position index (0 to 4)
        """
        # 1. Exploration Check
        if random.random() < self.epsilon:
            explore_pos = random.randint(0, 4)
            print(f"[Model] Exploration triggered: predicted random position {explore_pos}")
            return explore_pos

        # 2. Exploitation Check (Tabular Lookup)
        state_key = f"{s1_mask}_{s2_mask}"
        with self.lock:
            if state_key in self.model_data:
                predicted_pos = self.model_data[state_key]
                print(f"[Model] Tabular prediction hit: state {state_key} -> keeper position {predicted_pos}")
                return predicted_pos

        # 3. Physics Trajectory Extrapolator Prior (Zero-Shot Learning)
        c1 = get_active_col(s1_mask)
        c2 = get_active_col(s2_mask)
        
        if c1 != -1 and c2 != -1:
            # Extrapolate ball path: c3 = c2 + (c2 - c1) = 2 * c2 - c1
            predicted_pos = max(0, min(4, 2 * c2 - c1))
            print(f"[Model] Fallback to Physics Prior (both rows): S1={c1}, S2={c2} -> predicted position {predicted_pos}")
        elif c2 != -1:
            # If S1 was somehow skipped, default to matching the middle column
            predicted_pos = c2
            print(f"[Model] Fallback to Physics Prior (mid row only): S2={c2} -> predicted position {predicted_pos}")
        else:
            # Absolute fallback if no valid sensors are triggered
            predicted_pos = 2  # Center position
            print(f"[Model] Fallback to center: no valid sensors triggered -> predicted position {predicted_pos}")
            
        return predicted_pos

    def add_experience(self, s1_mask, s2_mask, s3_mask, was_goal, keeper_pos, on_train_complete=None):
        """
        add_experience
        Logs a shot experience sample and triggers asynchronous retraining.
        
        Why: Direct reinforcement feedback:
             - If a goal was scored: the ball passed at S3's active column (target = s3_col).
             - If a save was made: the keeper successfully intercepted the ball (target = keeper_pos).
             
        @param s1_mask: int - 5-bit trigger mask for Sensor Row 1
        @param s2_mask: int - 5-bit trigger mask for Sensor Row 2
        @param s3_mask: int - 5-bit trigger mask for Sensor Row 3
        @param was_goal: bool - True if ball scored, False if saved
        @param keeper_pos: int - Column index where the keeper was stationed
        @param on_train_complete: function - Callback to execute when async training completes
        """
        if was_goal:
            target_col = get_active_col(s3_mask)
            # If goal was scored but S3 mask is somehow 0, fallback to keeper_pos (or center)
            if target_col == -1:
                target_col = keeper_pos
        else:
            # Save: ball was blocked by the goalkeeper, meaning the target trajectory aligned with keeper position
            target_col = keeper_pos

        # Clamp target col to safety boundaries [0, 4]
        target_col = max(0, min(4, target_col))

        # Append to history database under reentrant lock
        with self.lock:
            self.history.append({
                "s1": s1_mask,
                "s2": s2_mask,
                "target": target_col
            })
            self.save_history()

        print(f"[Model] Logged experience (S1={s1_mask}, S2={s2_mask}, target={target_col}). Triggering async training thread...")
        
        # Run training asynchronously in a background thread to prevent blocking RPC execution.
        # This keeps the round-trip latency to the MCU minimal.
        training_thread = threading.Thread(target=self._train_async, args=(on_train_complete,))
        training_thread.daemon = True
        training_thread.start()

    def _train_async(self, on_train_complete=None):
        """
        _train_async
        Background training routine. Analyzes all historical logs to re-build the policy mapping.
        Updates model weights atomically when training finishes.
        """
        print("[Model-Train] Asynchronous training thread started.")
        try:
            # Safely copy history and parameters to run calculations lock-free, avoiding thread contention
            with self.lock:
                local_history = list(self.history)
                current_epsilon = self.epsilon

            # Build a frequency mapping dictionary: state -> [action_counts]
            # state_counts["s1_s2"] = [count_a0, count_a1, count_a2, count_a3, count_a4]
            state_counts = {}
            for shot in local_history:
                state_key = f"{shot['s1']}_{shot['s2']}"
                target = shot["target"]
                
                if state_key not in state_counts:
                    state_counts[state_key] = [0] * 5
                state_counts[state_key][target] += 1

            # Build the optimal policy mapping by choosing the action with maximum counts for each state
            new_model_data = {}
            for state_key, counts in state_counts.items():
                max_count = max(counts)
                # Filter indices achieving the maximum count. Ties are broken randomly to break systemic bias.
                best_actions = [action for action, count in enumerate(counts) if count == max_count]
                new_model_data[state_key] = random.choice(best_actions)

            # Decay epsilon (exploration probability) by 5% per training loop, floor at 5% (0.05)
            # Clamping prevents the goalkeeper from ever becoming 100% deterministic, allowing adaptability.
            new_epsilon = max(0.05, current_epsilon * 0.95)

            # Lock the active weights atomically and swap in the newly trained policy
            with self.lock:
                self.model_data = new_model_data
                self.epsilon = new_epsilon
                self.save_model()
            print(f"[Model-Train] Asynchronous training completed. Model size: {len(new_model_data)} states. Epsilon decayed to {new_epsilon:.3f}")
            
            # Trigger the optional post-training completion callback to sync policy weights to MCU
            if on_train_complete:
                try:
                    on_train_complete()
                except Exception as cb_err:
                    print(f"[Model-Train] Error executing on_train_complete callback: {cb_err}")
        except Exception as e:
            print(f"[Model-Train] Error in background training thread: {e}")
