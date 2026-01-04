import ext_plant
import ex2
import ex2_random
import sys
import time
import multiprocessing
from functools import partial

# ANSI color codes
RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
SEP = "\n" + ("=" * 60)

# --- WORKER FUNCTION FOR PARALLEL EXECUTION ---
def worker_task(args):
    """
    Runs a single simulation for a specific seed.
    Args:
        args: tuple containing (problem_config, seed, controller_mode, horizon, time_limit)
    Returns:
        (reward, duration, is_timeout)
    """
    problem_config, seed, controller_mode, horizon, time_limit = args
    
    # Ensure correct controller module is used
    if controller_mode == 'random':
        controller_module = ex2_random
    else:
        controller_module = ex2

    # Setup specific run
    current_problem = problem_config.copy()
    current_problem["seed"] = seed
    
    # Create game (suppress debug output)
    try:
        game = ext_plant.create_pressure_plate_game((current_problem, False))
    except Exception:
        # Fallback if creation fails
        return 0, 0, True

    policy = controller_module.Controller(game)
    
    # Timing execution
    start_time = time.time()
    
    try:
        max_steps = game.get_max_steps()
        for _ in range(max_steps):
            action = policy.choose_next_action(game.get_current_state())
            game.submit_next_action(action)
            if game.get_done():
                break
        
        reward = game.get_current_reward()
    except Exception:
        reward = 0
        
    duration = time.time() - start_time
    is_timeout = duration > time_limit
    
    return reward, duration, is_timeout

# --- PROBLEM DEFINITIONS ---
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
    "horizon": 30,
}

problem_pdf2 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {10: 0.9, 11: 0.8},
    "goal_reward": 12,
    "plants_reward": {(0, 2): [1, 3, 5, 7], (2, 0): [1, 2, 3, 4]},
    "seed": 45,
    "horizon": 35,
}

problem_pdf3 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {10: 0.7, 11: 0.6},
    "goal_reward": 30,
    "plants_reward": {(0, 2): [1, 2, 3, 4], (2, 0): [10, 11, 12, 13]},
    "seed": 45,
    "horizon": 30,
}

problem_new1_version1 = {
    "Size": (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
    "robot_chosen_action_prob": {10: 0.9, 11: 0.95},
    "goal_reward": 30,
    "plants_reward": {(4, 5): [1, 2, 3, 4], (0, 1): [10, 11, 12, 13]},
    "seed": 45,
    "horizon": 30,
}

problem_new1_version2 = {
    "Size": (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
    "robot_chosen_action_prob": {10: 0.6, 11: 0.95},
    "goal_reward": 30,
    "plants_reward": {(4, 5): [1, 2, 3, 4], (0, 1): [10, 11, 12, 13]},
    "seed": 45,
    "horizon": 70,
}

problem_new1_version3 = {
    "Size": (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 2, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
    "robot_chosen_action_prob": {10: 0.6, 11: 0.95},
    "goal_reward": 30,
    "plants_reward": {(4, 5): [1, 2, 3, 4], (0, 1): [10, 11, 12, 13]},
    "seed": 45,
    "horizon": 30,
}

problem_new2_version1 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.95},
    "goal_reward": 18,
    "plants_reward": {(0, 0): [5, 7], (4, 5): [5, 7]},
    "seed": 45,
    "horizon": 30,
}

problem_new2_version2 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.95},
    "goal_reward": 18,
    "plants_reward": {(0, 0): [5, 7], (4, 5): [5, 7]},
    "seed": 45,
    "horizon": 70,
}

problem_new2_version3 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.95},
    "goal_reward": 20,
    "plants_reward": {(0, 0): [5, 7, 9], (4, 5): [5, 7]},
    "seed": 45,
    "horizon": 30,
}

problem_new2_version4 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
    "robot_chosen_action_prob": {10: 0.7, 11: 0.95},
    "goal_reward": 18,
    "plants_reward": {(0, 0): [5, 7], (4, 5): [5, 7]},
    "seed": 45,
    "horizon": 40,
}

problem_new3_version1 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1), (1, 1), (2, 1), (3, 1), (4, 1),
        (6, 1), (7, 1), (8, 1), (9, 1),
        (4, 2), (4, 3), (6, 2), (6, 3),
    },
    "Taps": {(5, 3): 20},
    "Plants": {(0, 0): 10, (9, 0): 10},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 20)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.95},
    "goal_reward": 9,
    "plants_reward": {(0, 0): [1, 3], (9, 0): [1, 3]},
    "seed": 45,
    "horizon": 30,
}

problem_new3_version2 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1), (1, 1), (2, 1), (3, 1), (4, 1),
        (6, 1), (7, 1), (8, 1), (9, 1),
        (4, 2), (4, 3), (6, 2), (6, 3),
    },
    "Taps": {(5, 3): 20},
    "Plants": {(0, 0): 10, (9, 0): 10},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 20)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.8},
    "goal_reward": 9,
    "plants_reward": {(0, 0): [1, 3], (9, 0): [1, 3]},
    "seed": 45,
    "horizon": 50,
}

problem_new3_version3 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1), (1, 1), (2, 1), (3, 1), (4, 1),
        (6, 1), (7, 1), (8, 1), (9, 1),
        (4, 2), (4, 3), (6, 2), (6, 3),
    },
    "Taps": {(5, 3): 20},
    "Plants": {(0, 0): 5, (9, 0): 5},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 20)},
    "robot_chosen_action_prob": {10: 0.95, 11: 0.0001},
    "goal_reward": 9,
    "plants_reward": {(0, 0): [1, 3], (9, 0): [1, 3]},
    "seed": 45,
    "horizon": 70,
}

problem_new4_version1 = {
    "Size": (10, 10),
    "Walls": set(),
    "Taps": {(8, 8): 24},
    "Plants": {(0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5},
    "Robots": {10: (8, 9, 0, 5)},
    "robot_chosen_action_prob": {10: 0.95},
    "goal_reward": 9,
    "plants_reward": {(0, 0): [1, 3], (0, 9): [1, 3], (9, 0): [1, 3], (9, 9): [1, 3]},
    "seed": 45,
    "horizon": 70,
}

problem_new4_version2 = {
    "Size": (10, 10),
    "Walls": set(),
    "Taps": {(8, 8): 24},
    "Plants": {(0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5},
    "Robots": {10: (8, 9, 0, 5)},
    "robot_chosen_action_prob": {10: 0.85},
    "goal_reward": 9,
    "plants_reward": {(0, 0): [1, 3], (0, 9): [1, 3], (9, 0): [1, 3], (9, 9): [1, 3]},
    "seed": 45,
    "horizon": 40,
}


def main():
    # --- CONFIGURATION ---
    n_runs = 30
    args_cli = sys.argv[1:]
    controller_mode = "random" if "random" in args_cli else "normal"
    
    # List of problems to solve
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
    ]

    baseline_map = {
        "problem_pdf": (21.766667, 1.296914),
        "problem_pdf2": (33.566667, 1.173343),
        "problem_pdf3": (40.366667, 1.208422),
        "problem_new1_version1": (62.966667, 7.863858),
        "problem_new1_version2": (87.500000, 16.394542),
        "problem_new1_version3": (26.833333, 6.774192),
        "problem_new2_version1": (26.600000, 12.010865),
        "problem_new2_version2": (86.066667, 27.054848),
        "problem_new2_version3": (46.533333, 10.588311),
        "problem_new2_version4": (39.733333, 15.901740),
        "problem_new3_version1": (2.933333, 0.898599),
        "problem_new3_version2": (4.033333, 1.386519),
        "problem_new3_version3": (5.900000, 2.025747),
        "problem_new4_version1": (38.033333, 2.906778),
        "problem_new4_version2": (16.566667, 1.627962),
    }

    summaries = []
    
    # Determine CPU count for pool
    cpu_count = multiprocessing.cpu_count()
    print(f"{BOLD}Starting parallel execution on {cpu_count} cores...{RESET}")
    print(f"Solving {len(problems)} problems, {n_runs} runs each. Please wait.")
    
    # Create Pool
    with multiprocessing.Pool(processes=cpu_count) as pool:
        for idx, (pname, problem) in enumerate(problems, start=1):
            sys.stdout.write(f"\rProcessing problem {idx}/{len(problems)}: {pname:<25}")
            sys.stdout.flush()

            horizon = problem.get("horizon", 0)
            time_limit = 20 + 0.5 * horizon
            baseline_reward, baseline_time = baseline_map.get(pname, (None, None))

            # Prepare tasks
            tasks = []
            for seed in range(n_runs):
                tasks.append((problem, seed, controller_mode, horizon, time_limit))
            
            # Execute in parallel
            # map preserves order, but we don't strictly need it. 
            results = pool.map(worker_task, tasks)
            
            # Aggregate stats
            total_reward = sum(r[0] for r in results)
            # Duration is sum of individual run durations (to match baseline comparison metric)
            total_duration_metric = sum(r[1] for r in results)
            
            timeouts = sum(1 for r in results if r[2])
            
            avg_reward = total_reward / n_runs if n_runs else 0.0
            
            # Formulate status
            if timeouts == 0:
                time_status = f"{GREEN}PASS{RESET}"
            else:
                time_status = f"{RED}TIMEOUT ({timeouts}/{n_runs} runs){RESET}"

            summaries.append(
                (pname, avg_reward, total_duration_metric, baseline_reward, baseline_time, time_status)
            )

    print(f"\n\n{SEP}\n{BOLD}{CYAN}=== Summary (per problem) ==={RESET}")
    
    for pname, avg, dur, baseline_avg, baseline_time, t_status in summaries:
        avg_col = f"{GREEN}{avg:.2f}{RESET}" if avg >= 0 else f"{RED}{avg:.2f}{RESET}"
        
        if baseline_avg is not None:
            pct = (avg / baseline_avg * 100) if baseline_avg > 0 else 0
            comp = (
                f"{GREEN}BETTER{RESET}" if avg > baseline_avg else f"{RED}WORSE{RESET}"
            )
            baseline_str = f"baseline {YELLOW}{baseline_avg:.2f}{RESET} (time {YELLOW}{baseline_time:.2f}s{RESET})"
        else:
            pct = 0
            comp = ""
            baseline_str = "no baseline"
            
        print(
            f"{BOLD}{pname}{RESET}: average = {avg_col} ({pct:.1f}% of {baseline_str}) | time = {YELLOW}{dur:.2f}s{RESET} | {comp} {t_status}"
        )

    total_time_metric = sum(d for (_, _, d, _, _, _) in summaries)
    print(f"{BOLD}Total metric time (sum of all runs): {RESET}{YELLOW}{total_time_metric:.2f}s{RESET}")
    
    if summaries:
        overall_avg = sum(avg for (_, avg, _, _, _, _) in summaries) / len(summaries)
        print(f"{BOLD}Overall average reward across {len(summaries)} problems: {RESET}{YELLOW}{overall_avg:.2f}{RESET}")

if __name__ == "__main__":
    multiprocessing.freeze_support() # For Windows support
    main()
