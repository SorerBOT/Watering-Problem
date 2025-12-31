import time

import ex1
import search

VERBOSE = False

def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    result = (-3, "default")
    try:
        result = func(*targs, **kwargs)

    except Exception as e:
        result = (-3, e)
    return result
# check_problem: problem, search_method, timeout
# timeout_exec: search_method, targs=[problem], timeout_duration=timeout
def solve_problems(problem, algorithm):
    try:
        p = ex1.create_watering_problem(problem)
    except Exception as e:
        print("Error creating problem: ", e)
        return None
    if algorithm == "gbfs":
        result = run_problem((lambda p: search.greedy_best_first_graph_search(p, p.h_gbfs)),targs=[p])
    else:
        result = run_problem((lambda p: search.astar_search(p, p.h_astar)), targs=[p])

    if result and isinstance(result[0], search.Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        print(solution)
        return len(solution)
    else:
        return None


# ---------------------------------
# Problems
# ---------------------------------

# Successor check problems
problem_blocked_center_3x3 = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (1, 0), (1, 2), (2, 1)},
    "Taps":   {},
    "Plants": {},
    "Robots": {10: (1, 1, 0, 2)},
}

problem_1x1_out_of_bounds = {
    "Size":   (1, 1),
    "Walls":  set(),
    "Taps":   {},
    "Plants": {},
    "Robots": {10: (0, 0, 0, 2)},
}

problem_1x2_two_robots_no_overlap = {
    "Size":   (1, 2),
    "Walls":  set(),
    "Taps":   {},
    "Plants": {},
    "Robots": {10: (0, 0, 0, 2), 11: (0, 1, 0, 2)},
}

# Dead-end problems
problem_no_enough_water = {
    "Size":   (2, 2),
    "Walls":  set(),
    "Taps":   {(1, 0): 2},
    "Plants": {(0, 1): 10},
    "Robots": {10: (0, 0, 0, 10)},
}

problem_isolated_plant = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (1, 2)},
    "Taps":   {(2, 0): 20},
    "Plants": {(0, 2): 3},
    "Robots": {10: (2, 2, 0, 5)},
}

# Test problems (A* must be optimal)
# Optimal : 8
problem1 = {
    "Size":   (3, 3),
    "Walls":  set(),
    "Taps":   {(1, 1): 3},
    "Plants": {(0, 2): 2},
    "Robots": {10: (2, 0, 0, 2)},
}

# Optimal: 20
problem2 = {
    "Size":  (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps":  {(1, 1): 6},
    "Plants": {(0, 2): 3, (2, 0): 2},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

# optimal: 28
problem3 = {
    "Size":  (5, 3),
    "Walls": {(1, 1), (3, 1)},
    "Taps":  {(0, 0): 5},
    "Plants": {(4, 2): 4},
    "Robots": {10: (2, 0, 0, 2)},
}

# optimal: 13
problem4 = {
    "Size":  (5, 5),
    "Walls": {(0, 1),(1, 1),(2, 1), (0, 3),(1, 3),(2, 3)},
    "Taps": {(3, 2): 1, (4, 2): 1},
    "Plants": {(0, 2): 1, (1, 2): 1},
    "Robots": {10: (3, 1, 0, 1), 11: (3, 3, 0, 1)},
}

# optimal: 8
problem5 = {
    "Size":  (8, 8),
    "Walls": set(
        (r, c)
        for r in range(8)
        for c in range(8)
        if not (r == 1 and c in (0, 1, 2))
    ),
    "Taps": {(1, 1): 3},
    "Plants": {(1, 2): 3},
    "Robots": {10: (1, 0, 0, 3)},
}

# optimal: 21
problem6 = {
    "Size":  (4, 4),
    "Walls": set(),
    "Taps": {(2, 2): 18},
    "Plants": {(0, 3): 3, (3, 0): 3},
    "Robots": {10: (2, 1, 0, 3), 11: (2, 0, 0, 3)},
}

# Bench problems
# optimal: 76
problem_8 = {
    "Size":  (6, 5),
    "Walls": {(2, 2), (3, 2)},
    "Taps": {(1, 1): 15},
    "Plants": {(0, 4): 8, (5, 0): 4},
    "Robots": {11: (4, 3, 0, 2)},
}

problem_hard1 = {
    "Size":  (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
}

problem_hard6 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
}

problem_hard7 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 6, (4, 5): 6},
    "Robots": {10: (1, 1, 0, 6), 11: (3, 4, 0, 5)},
}

problem_12x12_snake_hard = {
    "Size":  (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3),
        (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),

        (1, 6), (3, 6), (4, 6), (5, 6),
        (6, 6), (7, 6), (8, 6), (10, 6),

        (1, 9), (2, 9), (3, 9), (4, 9),
        (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 2)},
}

problem_check = {
    "Size":  (8, 3),
    "Walls": {
        (1,0),(2, 0), (3, 0), (4, 0), (5, 0),
        (6, 0),(1,2),(2, 2), (3, 2), (4, 2), (5, 2),
        (6, 2),
    },
    "Taps": {(7, 1): 30},
    "Plants": {(0, 1): 15, (1, 1): 15},
    "Robots": {10: (7, 0, 0, 15), 11: (7, 2, 0, 15)},
}

# 48
problem_harder3 = {
    "Size": (8, 6),
    "Walls": {
        (1, 1), (1, 2), (1, 3), (1, 4),
        (3, 1), (3, 2), (3, 3), (3, 4),
        (5, 1), (5, 2), (5, 3), (5, 4),
        (6, 2), (6, 3),
    },
    "Taps": {(1, 5): 18, (7, 5): 18},
    "Plants": {(0, 5): 6, (7, 4): 6, (2, 5): 6},
    "Robots": {10: (0, 1, 0, 6), 11: (7, 4, 0, 3)},
}

# 97
problem_harder4 = {
    "Size": (2, 8),
    "Walls": {(1, 0), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7)},
    "Taps": {(0, 3): 40},
    "Plants": {(0, 0): 35, (0, 7): 5},
    "Robots": {10: (0, 7, 0, 35), 11: (0, 3, 0, 5)},
}

# 93
problem_harder66 = {
    "Size": (2, 8),
    "Walls": {(1, 0), (1, 2), (1, 3), (1, 4), (1, 5)},
    "Taps": {(0, 3): 40},
    "Plants": {(0, 0): 35, (0, 7): 5},
    "Robots": {10: (0, 7, 0, 35), 11: (0, 3, 0, 5)},
}

# 34
problem_harder666 = {
    "Size": (52, 52),
    "Walls": set(),
    "Taps": {(0, 1): 10, (51, 50): 2},
    "Plants": {(51, 49): 2, (0, 0): 10},
    "Robots": {10: (51, 50, 0, 10), 11: (0, 1, 0, 2)},
}

# 52
problem_harder44 = {
    "Size": (10, 6),
    "Walls": {
        (1, 2), (1, 3),
        (2, 2), (2, 3),
        (4, 1), (4, 2), (4, 3), (4, 4),
        (6, 2), (6, 3),
        (7, 2), (7, 3),
        (8, 4), (8, 5),
    },
    "Taps": {(0, 3): 10, (9, 2): 10},
    "Plants": {(0, 0): 2, (9, 5): 2, (5, 5): 1},
    "Robots": {10: (2, 0, 0, 1), 11: (7, 5, 0, 1)},
}

# optimal 85
problem_load_hard = {
    "Size":  (10, 4),
    "Walls": {
        (0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1),
        (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3)
    },
    "Taps": {(5, 3): 30},
    "Plants": {(0, 0): 15, (9, 0): 15},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 30)},
}

correct_answers = [8, 20, 28, 13, 8, 21, 76, 48, 97, 93, 34, 52, 85]

def main():
    problems = [(problem_harder4,            "problem_harder4",      97)]
    i = 1
    is_failed = False
    start = time.time()
    for (problem, problem_name, expected_len) in problems:
        print("------------------------------------------")
        for a in ['astar','gbfs']:
            print(f"Running {problem_name} with {a}")
            algorithm_time_start    = time.time()
            len                     = solve_problems(problem, a)
            algorithm_time_end      = time.time()

            if a == 'astar':
                if len != expected_len:
                    is_failed           = True
                    print(f"{problem_name} FAILED. Took: {len} steps instead of {expected_len}")
                else:
                    print(f"PASSED {problem_name} in {algorithm_time_end - algorithm_time_start} seconds")
            else:
                print(f"Solved {problem_name} GBFS. {len} steps solution (instead of {expected_len}) was found in {algorithm_time_end - algorithm_time_start} seconds")
        i += 1
    print("------------------------------------------")
    print(f"Final Result: {'SOME FAILED' if is_failed else 'ALL PASSED'}")
    end = time.time()
    print('Submission took:', end-start, 'seconds.')


if __name__ == '__main__':
    main()
