import ext_plant
import ex3
import time
import numpy as np
import ex3_random
import sys
import multiprocessing
from functools import partial

# ANSI color codes for nicer terminal output
RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
SEP = "\n" + ("=" * 60)


def draw_board(problem):
    """Draw a simple emoji board for the given problem dict."""
    rows, cols = problem.get("Size", (0, 0))
    walls = set(problem.get("Walls", []))
    taps = dict(problem.get("Taps", {}))
    plants = dict(problem.get("Plants", {}))
    robots = dict(problem.get("Robots", {}))

    # Build grid rows (r from 0..rows-1)
    grid_lines = []
    for r in range(rows):
        row_cells = []
        for c in range(cols):
            pos = (r, c)
            if pos in walls:
                cell = "🧱 "
            elif pos in taps:
                # show tap emoji
                cell = "🚰 "
            elif pos in plants:
                cell = "🌱 "
            else:
                cell = "⬜ "

            # overlay robot if present (show last digit of id)
            robot_here = None
            for rid, (rr, cc, _carried, _cap) in robots.items():
                if (rr, cc) == pos:
                    robot_here = rid
                    break
            if robot_here is not None:
                cell = f"🤖{str(robot_here)[-1]}"

            row_cells.append(cell)
        grid_lines.append(" ".join(row_cells))

    print("\nBoard layout:")
    for line in grid_lines:
        print(line)
    # also print legends for taps/plants/robots (counts)
    if taps:
        taps_str = ", ".join([f"{pos}:{amt}" for pos, amt in taps.items()])
        print(f"Taps: {taps_str}")
    if plants:
        plants_str = ", ".join([f"{pos}:{need}" for pos, need in plants.items()])
        print(f"Plants: {plants_str}")
    if robots:
        robots_str = ", ".join(
            [f"{rid}:{(r,c)}" for rid, (r, c, _car, _cap) in robots.items()]
        )
        print(f"Robots: {robots_str}")


def is_action_legal(game: ext_plant.Game, chosen_action: str):
    """Return (True, '') if action legal in current state, else (False, reason)."""
    if chosen_action == "RESET":
        return True, ""

    try:
        action_name, robot_id = game.parse_robot_action(chosen_action)
    except Exception as e:
        return False, f"Bad action format or unknown action: {e}"

    robots_t, plants_t, taps_t, total_water_need = game.get_current_state()

    robot_idx = None
    r = c = load = None
    for idx, (rid, (rr, cc), l) in enumerate(robots_t):
        if rid == robot_id:
            robot_idx = idx
            r, c, load = rr, cc, l
            break
    if robot_idx is None:
        return False, f"Robot {robot_id} not found in state"

    robot_pos = (r, c)
    plant_positions = {pos for (pos, need) in plants_t}
    tap_positions = {pos for (pos, water) in taps_t}
    occupied_positions = {
        (rr, cc) for (rid, (rr, cc), l) in robots_t if rid != robot_id
    }

    base_applicable = game.applicable_actions.get(robot_pos, [])
    dynamic_applicable = []
    for a in base_applicable:
        if a in ("UP", "DOWN", "LEFT", "RIGHT"):
            if a == "UP":
                target = (r - 1, c)
            elif a == "DOWN":
                target = (r + 1, c)
            elif a == "LEFT":
                target = (r, c - 1)
            else:
                target = (r, c + 1)
            if target in occupied_positions:
                continue
        dynamic_applicable.append(a)

    if action_name in ("UP", "DOWN", "LEFT", "RIGHT"):
        if action_name not in dynamic_applicable:
            return (
                False,
                f"Move {action_name} not applicable from {robot_pos} (blocked or wall)",
            )
        return True, ""
    if action_name == "POUR":
        if action_name not in dynamic_applicable:
            return False, "POUR not applicable at this position"
        if load <= 0 or robot_pos not in plant_positions:
            return False, "POUR precondition failed: not standing on plant or no load"
        return True, ""
    if action_name == "LOAD":
        if action_name not in dynamic_applicable:
            return False, "LOAD not applicable at this position"
        cap = game.get_capacities().get(robot_id, None)
        if cap is None:
            return False, "Could not get robot capacity"
        if load >= cap or robot_pos not in tap_positions:
            return False, "LOAD precondition failed: at tap or capacity reached"
        return True, ""

    return False, "Unknown action or not applicable"


# --- WORKER FUNCTION FOR PARALLEL EXECUTION ---
def run_single_seed(seed, problem, controller_cls, debug_mode, time_limit, strict_mode):
    """
    Worker function to run a single seed.
    Returns: (reward, duration, success_flag, error_message)
    """
    # Create a fresh copy of the problem with the specific seed
    # We must do this because 'problem' is passed across process boundaries
    current_problem = problem.copy()
    current_problem["seed"] = seed

    # Re-import ext_plant inside process if needed, but usually fine globally
    game = ext_plant.create_pressure_plate_game((current_problem, debug_mode))
    
    # Initialize controller
    try:
        policy = controller_cls(game)
    except Exception as e:
        return 0.0, 0.0, False, f"Init Error: {str(e)}"

    start_time = time.time()
    
    try:
        for i in range(game.get_max_steps()):
            action = policy.choose_next_action(game.get_current_state())
            
            # Check legality
            legal, reason = is_action_legal(game, action)
            if not legal:
                return 0.0, time.time() - start_time, False, f"Illegal Action: {action} ({reason})"
            
            game.submit_next_action(chosen_action=action)
            
            # Check timeout inside loop if strict (optional, but good for long hangs)
            if strict_mode and (time.time() - start_time > time_limit):
                 return game.get_current_reward(), time.time() - start_time, False, "TIMEOUT_INTERRUPT"

            if game.get_done():
                break
                
        duration = time.time() - start_time
        reward = game.get_current_reward()
        
        # Check if goal reached (optional logic depending on how you define success)
        # Here we just return True for 'ran without error'
        return reward, duration, True, None

    except Exception as e:
        return 0.0, time.time() - start_time, False, f"Runtime Error: {str(e)}"


# -----------------------------------------------

# [INSERT PROBLEM DICTIONARIES HERE - keeping them abbreviated for length]
# Copy the full dictionaries (problem_pdf ... problem8) from your original file here.
# For brevity in this response, I assume they are defined above or imported.
# ... (problem definitions) ...
problem_pdf = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.9},
    "goal_reward": 10,
    "plants_reward": {(0, 2): [1, 2, 3, 4], (2, 0): [1, 2, 3, 4]},
    "seed": 45,
    "horizon": 60,
}
# ... (Assume all other problems are present as in original file) ...
# To ensure the code runs, I will include just the list setup, 
# you should paste your problem dicts back in.

problem_pdf2 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {
        10: 0.9,
        11: 0.8,
    },
    "goal_reward": 12,
    "plants_reward": {
        (0, 2): [1, 3, 5, 7],
        (2, 0): [1, 2, 3, 4],
    },
    "seed": 45,
    "horizon": 60,
}

problem_pdf3 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {
        10: 0.7,
        11: 0.6,
    },
    "goal_reward": 30,
    "plants_reward": {
        (0, 2): [1, 2, 3, 4],
        (2, 0): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 60,
}

problem_new1_version1 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 3,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.9,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 60,
}
problem_new1_version2 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 3,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.6,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 70,
}
problem_new1_version3 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 2,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.6,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 60,
}

problem_new2_version1 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 60,
}

problem_new2_version2 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 70,
}
problem_new2_version3 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 20,
    "plants_reward": {
        (0, 0): [5, 7, 9],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 60,
}
problem_new2_version4 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.7,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 40,
}


problem_new3_version1 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 10,  # upper-right corrido
        (9, 0): 10,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 60,
}

problem_new3_version2 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 10,  # upper-right corrido
        (9, 0): 10,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.8,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 100,
}


problem_new3_version3 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 5,  # upper-right corrido
        (9, 0): 5,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.0001,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 210,  # "horizon": 70,
}
# reset ?
problem_new4_version1 = {
    "Size": (10, 10),
    "Walls": set(),  # completely open grid
    "Taps": {
        (8, 8): 24,
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (0, 9): 5,  # top-right
        (9, 0): 5,  # bottom-left
        (9, 9): 5,  # bottom-right
        # total need = 20
    },
    "Robots": {
        10: (8, 9, 0, 5),
    },
    "robot_chosen_action_prob": {
        10: 0.95,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (0, 9): [1, 3],
        (9, 0): [1, 3],
        (9, 9): [1, 3],
    },
    "seed": 45,
    "horizon": 140,  # "horizon": 70,
}

# reset ?
problem_new4_version2 = {
    "Size": (10, 10),
    "Walls": set(),  # completely open grid
    "Taps": {
        (8, 8): 24,
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (0, 9): 5,  # top-right
        (9, 0): 5,  # bottom-left
        (9, 9): 5,  # bottom-right
        # total need = 20
    },
    "Robots": {
        10: (8, 9, 0, 5),
    },
    "robot_chosen_action_prob": {
        10: 0.85,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (0, 9): [1, 3],
        (9, 0): [1, 3],
        (9, 9): [1, 3],
    },
    "seed": 45,
    "horizon": 80,
}


# ⬜ problems (copied from ex2_check.py) — horizons and boundlines from image
problem1 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    "Taps": {(5, 3): 30},
    "Plants": {(0, 0): 15, (9, 0): 15},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 30)},
    "robot_chosen_action_prob": {
        10: 0.9,
        11: 0.9,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 43,
    "boundline": 2.742154,
}

problem2 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    "Taps": {(5, 3): 30},
    "Plants": {(0, 0): 13, (9, 0): 13},
    "Robots": {10: (2, 0, 0, 15), 11: (7, 0, 0, 15)},
    "robot_chosen_action_prob": {
        10: 0.4,
        11: 0.95,
    },
    "goal_reward": 24,
    "plants_reward": {
        (0, 0): [10, 12],
        (9, 0): [1, 2],
    },
    "seed": 45,
    "horizon": 75,
    "boundline": 5.934166,
}

problem3 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    "Taps": {(5, 3): 40},
    "Plants": {(0, 0): 10, (9, 0): 18},
    "Robots": {11: (7, 0, 0, 40)},
    "robot_chosen_action_prob": {
        11: 0.95,
    },
    "goal_reward": 24,
    "plants_reward": {
        (0, 0): [10, 12],
        (9, 0): [1, 2],
    },
    "seed": 45,
    "horizon": 100,
    "boundline": 4.234852,
}

problem4 = {
    "Size": (6, 6),
    "Walls": {(0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (5, 0), (4, 0), (3, 0), (2, 0)},
    "Taps": {(0, 1): 2, (0, 4): 11},
    "Plants": {(0, 0): 5, (0, 5): 8},
    "Robots": {10: (1, 1, 0, 10), 11: (1, 5, 0, 10)},
    "robot_chosen_action_prob": {
        10: 0.4,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (0, 0): [10, 11, 12],
        (0, 5): [2, 3, 4],
    },
    "seed": 45,
    "horizon": 100,  # updated per image
    "boundline": 4.327797,
}

problem5 = {
    "Size": (5, 4),
    "Walls": {
        (3, 0),
        (3, 1),
        (3, 2),
    },
    "Taps": {(1, 2): 7, (1, 3): 8},
    "Plants": {(0, 1): 3, (0, 3): 3, (4, 0): 9},
    "Robots": {10: (2, 2, 0, 6), 11: (4, 1, 0, 10)},
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 35,
    "plants_reward": {
        (0, 1): [2, 3, 4],
        (0, 3): [2, 3, 4],
        (4, 0): [20, 22],
    },
    "seed": 45,
    "horizon": 60,  # updated per image
    "boundline": 5.896793,
}

problem6 = {
    "Size": (2, 2),
    "Walls": {},
    "Taps": {(0, 1): 20},
    "Plants": {(0, 0): 10, (1, 1): 10},
    "Robots": {
        10: (0, 1, 0, 12),
    },
    "robot_chosen_action_prob": {
        10: 0.95,
    },
    "goal_reward": 35,
    "plants_reward": {
        (0, 0): [10, 12],
        (1, 1): [1, 2],
    },
    "seed": 45,
    "horizon": 100,
    "boundline": 37.17993,
}

problem7 = {
    "Size": (4, 4),
    "Walls": {},
    "Taps": {(0, 1): 100},
    "Plants": {(0, 0): 100},
    "Robots": {
        10: (0, 1, 0, 100),
    },
    "robot_chosen_action_prob": {
        10: 0.95,
    },
    "goal_reward": 4,
    "plants_reward": {
        (0, 0): [1, 2],
    },
    "seed": 45,
    "horizon": 60,  # updated per image
    "boundline": 2.095383,
}

problem8 = {
    "Size": (4, 4),
    "Walls": {},
    "Taps": {(0, 1): 10, (2, 3): 5},
    "Plants": {
        (0, 0): 8,
        (3, 3): 7,
    },
    "Robots": {
        10: (1, 1, 0, 10),
        11: (2, 2, 0, 100),
    },
    "robot_chosen_action_prob": {10: 0.95, 11: 0.95},
    "goal_reward": 4,
    "plants_reward": {
        (0, 0): [1, 2],
        (3, 3): [1, 2],
    },
    "seed": 45,
    "horizon": 60,  # updated per image
    "boundline": 1.080202,
}


def main():
    debug_mode = False
    n_runs = 30
    
    # Official baselines
    official_baseline_map = {
        "problem_pdf": (44.0, 0.0),
        "problem_pdf3": (37.0, 0.0),
        "problem_new1_version1": (83.0, 0.0),
        "problem_new1_version3": (57.0, 0.0),
        "problem_new2_version1": (85.0, 0.0),
        "problem_new2_version3": (88.0, 0.0),
        "problem_new3_version1": (18.0, 0.0),
        "problem_new3_version2": (20.0, 0.0),
        "problem_new4_version1": (35.0, 0.0),
        "problem_new4_version2": (12.0, 0.0),
    }

    # Estimated baselines
    estimated_baseline_map = {
        "problem_pdf2": (28.33333, 0.0),
        "problem_new1_version2": (81.87778, 0.0),
        "problem_new2_version2": (123.715, 0.0),
        "problem_new2_version4": (41.65, 0.0),
        "problem_new3_version3": (8.922222, 0.0),
        "problem1": (50.0, 5.0),
        "problem2": (120.0, 10.0),
        "problem3": (160.0, 2.0),
        "problem4": (65.0, 15.0),
        "problem5": (75.0, 10.0),
        "problem6": (300.0, 30.0),
        "problem7": (22.0, 1.0),
        "problem8": (28.0, 5.0),
    }
    
    problems = [
        ("problem_pdf", problem_pdf),
        ("problem_pdf2", problem_pdf2),
        ("problem_pdf3", problem_pdf3),
        ("problem_new1_version1", problem_new1_version1),
        ("problem_new1_version2", problem_new1_version2),
        ("problem_new1_version3", problem_new1_version3),
        ("problem_new2_version1", problem_new2_version1),
        ("problem_new2_version2", problem_new2_version2),
        ("problem_new2_version3", problem_new2_version3),
        ("problem_new2_version4", problem_new2_version4),
        ("problem_new3_version1", problem_new3_version1),
        ("problem_new3_version2", problem_new3_version2),
        ("problem_new3_version3", problem_new3_version3),
        ("problem_new4_version1", problem_new4_version1),
        ("problem_new4_version2", problem_new4_version2),
        ("problem1", problem1),
        ("problem2", problem2),
        ("problem3", problem3),
        ("problem4", problem4),
        ("problem5", problem5),
        ("problem6", problem6),
        ("problem7", problem7),
        ("problem8", problem8),
    ]

    args = sys.argv[1:]
    argstr = " ".join(args).lower()
    
    # Flags
    baseline_only = "baseline" in argstr
    strict_mode = "strict" in argstr
    
    # Controller
    if "random" in args:
        controller_module = ex3_random
        controller_cls = controller_module.Controller
    else:
        controller_module = ex3
        controller_cls = controller_module.Controller

    # Filter Problems
    selected_problem_names = None
    all_problem_names = [p for (p, _) in problems]
    for a in args:
        if a.startswith("[") and a.endswith("]"):
            inner = a[1:-1]
            names = [x.strip() for x in inner.split(",") if x.strip()]
            if names:
                selected_problem_names = names
            break

    if selected_problem_names is not None:
        problems = [(p, prob) for (p, prob) in problems if p in selected_problem_names]
        missing = [n for n in selected_problem_names if n not in all_problem_names]
        if missing:
            print(f"{YELLOW}Warning: requested problems not found: {missing}{RESET}")
    elif baseline_only:
        problems = [(p, prob) for (p, prob) in problems if p in official_baseline_map]

    print(f"{BOLD}{YELLOW}Running {len(problems)} problems in PARALLEL (Pool Size: {multiprocessing.cpu_count()}){RESET}")

    summaries = []

    # Helper function for baseline lookup
    def get_baseline_info(pname: str):
        if pname in official_baseline_map:
            return official_baseline_map[pname] + (False,)
        if pname in estimated_baseline_map:
            return estimated_baseline_map[pname] + (True,)
        return None, None, False

    # --- MAIN LOOP ---
    for idx, (pname, problem) in enumerate(problems, start=1):
        marker = "🟩" if pname in official_baseline_map else "⬜"
        print(f"\n{marker} *** Problem: {pname} ({idx}) ***")
        draw_board(problem)
        print(f"{SEP}\n{BOLD}{MAGENTA}--- Running {pname} (Parallel {n_runs} seeds) ---{RESET}")

        horizon = problem.get("horizon", 0)
        time_limit = 20 + 0.5 * horizon
        
        # Prepare arguments for map
        # We need to bind specific args to the function to map over just the seed
        worker_func = partial(run_single_seed, 
                              problem=problem, 
                              controller_cls=controller_cls, 
                              debug_mode=debug_mode, 
                              time_limit=time_limit,
                              strict_mode=strict_mode)

        run_start_total = time.time()
        
        # --- PARALLEL EXECUTION ---
        with multiprocessing.Pool() as pool:
            # results: list of (reward, duration, success, msg)
            results = pool.map(worker_func, range(n_runs))
            
        total_duration_real_time = time.time() - run_start_total

        # Aggregate Results
        total_reward = 0.0
        total_cpu_time = 0.0
        timeouts = 0
        
        for seed, (r, dur, success, msg) in enumerate(results):
            if not success:
                print(f"{RED}Run {seed} FAILED: {msg}{RESET}")
                if strict_mode:
                    print(f"{RED}Strict mode enabled. Aborting script.{RESET}")
                    sys.exit(1)
            
            total_reward += r
            total_cpu_time += dur
            
            # Print brief status for each run (optional, can be noisy)
            status_color = GREEN if dur <= time_limit else RED
            limit_str = "OK" if dur <= time_limit else "TIMEOUT"
            if dur > time_limit: timeouts += 1
            
            # Uncomment below to see every seed's result line (spammy in parallel)
            # print(f"Run {seed:2d}: Reward {r:3d} | Time {dur:.2f}s | {status_color}{limit_str}{RESET}")

        avg_reward = total_reward / n_runs
        avg_time_per_run = total_cpu_time / n_runs
        
        time_status = f"{GREEN}PASS{RESET}" if timeouts == 0 else f"{RED}TIMEOUT ({timeouts}/{n_runs}){RESET}"
        
        # Comparison logic
        baseline_avg, baseline_time, is_estimated = get_baseline_info(pname)
        
        summaries.append((pname, avg_reward, total_duration_real_time, baseline_avg, baseline_time, is_estimated, time_status))

        print(f"Total Wall Time: {total_duration_real_time:.2f}s (Avg CPU time/run: {avg_time_per_run:.2f}s)")
        
        if baseline_avg is not None:
            pct = (avg_reward / baseline_avg * 100) if baseline_avg > 0 else 0
            comp = f"{GREEN}BETTER{RESET}" if avg_reward > baseline_avg else f"{RED}WORSE{RESET}"
            label = "estimated baseline" if is_estimated else "baseline"
            print(f"Average Reward: {avg_reward:.2f} ({pct:.1f}% of {label} {baseline_avg:.2f}) | {comp}")
        else:
             print(f"Average Reward: {avg_reward:.2f}")
             
        print(f"Status: {time_status}")


    # --- FINAL SUMMARY ---
    print(f"\n{SEP}\n{BOLD}{CYAN}=== Summary (per problem) ==={RESET}")
    for (pname, avg, dur, baseline_avg, baseline_time, is_estimated, t_status) in summaries:
        avg_col = f"{GREEN}{avg:.2f}{RESET}" if avg >= 0 else f"{RED}{avg:.2f}{RESET}"
        
        if baseline_avg is not None:
            pct = (avg / baseline_avg * 100) if baseline_avg > 0 else 0
            comp = f"{GREEN}BETTER{RESET}" if avg > baseline_avg else f"{RED}WORSE{RESET}"
            bl_label = "est. base" if is_estimated else "base"
            baseline_str = f"{bl_label} {YELLOW}{baseline_avg:.2f}{RESET}"
        else:
            comp = ""
            baseline_str = "no baseline"
            
        print(f"{BOLD}{pname}{RESET}: avg {avg_col} | {baseline_str} | Wall Time {YELLOW}{dur:.2f}s{RESET} | {comp} {t_status}")

if __name__ == "__main__":
    # Windows support for multiprocessing
    multiprocessing.freeze_support()
    main()
