import ext_plant
import numpy as np
import heapq
import re

# AI utilisation:
# To the predominant majority of chunks of code in this file, in which are present
# lines of code that might chance to be found incoherent, I have attached comments
# aimed to attribute the present logic to a certain form of calculation, or deduction.
# such comments in which my reasoning is described at length, were sent as prompts
# to various LLMs in order to have them validated. In most instances, I was congratulated
# for my sound logic, but at times, my logic was reproached and I have accounted for that
# in the code. Apart from the aforementioned interactions, not a single line of code
# has happened to be borrowed from any such LLM, but they are all the fruits of my
# own labouring.

# Consulting other students
# Although I did consult other students, our discussions were vastly infertile
# as we have each chosen a direction inherently different from the others
# and so we could not but suggest the most trivial of ameliorations to one anther.

id = [""]

ACTIONS = {"LOAD", "UP", "RIGHT", "LEFT", "DOWN", "POUR"}
Cords = tuple[int, int]
Robot = tuple[int, Cords, int]
Plant = tuple[Cords, int]
Tap   = tuple[Cords, int]

class Controller:
    """This class is a controller for the ext_plant game."""

    original_game: ext_plant.Game
    bfs_distances: dict[tuple[Cords, Cords], int]
    bfs_paths:          dict[tuple[ tuple[int, int], tuple[int, int] ], set[tuple[int, int]]]


    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game
        self.bfs_paths = {}
        self.bfs_distances = {}
        self.plant_rewards = dict([(plant_cords, 5) for plant_cords in game.plants])
        self.best_run_length = 0
        self.best_run_mean_reward_per_step = 0
        self.current_run_mean_reward = 0
        self.current_run_length = 0
        self.is_in_run = False
        self.robot_chosen_action_prob: dict[int, tuple[int, float]] = dict( (r_id, (1, 1)) for r_id in game._robots.keys())
        self.last_action = None
        self.previous_state = None

    # yoinked from ext_plant.py
    def parse_robot_action(self, s: str):
        m = re.fullmatch(r"\s*([A-Za-z_]\w*)\s*\(\s*([^)]+?)\s*\)\s*", s)
        if not m:
            raise ValueError(f"Bad action format: {s!r}")

        action = m.group(1).upper()
        if action not in ACTIONS:
            raise ValueError(f"Unknown action {action!r} in {s!r}")

        robot_id_str = m.group(2).strip()
        robot_id = int(robot_id_str)  # will raise if not an integer

        return action, robot_id

    def get_robot_pos_in_state(self, robot_id, robots):
        robot_search_res = [robot for robot in robots if robot[0] == robot_id]
        robot = robot_search_res[0]
        robot_id, (r, c), load = robot
        return (r,c)

    def get_plant_water_count(self, cords, plants):
        plant_res = [plant for plant in plants if plant[0] == cords]
        if len(plant_res) == 0:
            return 0
        plant = plant_res[0]
        return plant[1]

    def get_tap_water_count(self, cords, taps):
        tap_res = [tap for tap in taps if tap[0] == cords]
        if len(tap_res) == 0:
            return None
        tap = tap_res[0]
        return tap[1]

    def check_if_last_action_succeeded(self, current_state):
        if self.previous_state is None:
            return
        last_action = self.last_action
        if (last_action == "RESET"):
            return

        (previous_robots, previous_plants, previous_taps, ___) = self.previous_state
        (current_robots, current_plants, current_taps, ___a) = current_state

        action, robot_id = self.parse_robot_action(last_action)
        (r_p, c_p) = self.get_robot_pos_in_state(robot_id, previous_robots)
        (r, c) = self.get_robot_pos_in_state(robot_id, current_robots)

        is_success = 0
        if action == "LEFT":
            is_success = r == r_p and c_p - c == 1
        elif action == "RIGHT":
            is_success = r == r_p and c - c_p == 1
        elif action == "UP":
            is_success = r == r_p - 1 and c == c_p
        elif action == "DOWN":
            is_success = r == r_p + 1 and c == c_p
        elif action == "LOAD":
            previous_water_count = self.get_tap_water_count((r_p, c_p), previous_taps)
            current_water_count = self.get_tap_water_count((r_p, c_p), current_taps)
            if current_water_count is None:
                is_success = True
            else:
                is_success = previous_water_count - current_water_count == 1
        elif action == "POUR":
            previous_water_count = self.get_plant_water_count((r_p, c_p), previous_plants)
            current_water_count = self.get_plant_water_count((r_p, c_p), current_plants)
            if current_water_count is None:
                is_success = True
            else:
                is_success = previous_water_count - current_water_count == 1

        self.update_probability(robot_id, is_success)

    def update_probability(self, robot_id, is_success):
        current_state = self.robot_chosen_action_prob[robot_id]
        if current_state == None:
            self.robot_chosen_action_prob[robot_id] = (1, 1 if is_success else 0.25)
        else:
            (steps_count, current_success_rate) = current_state
            new_success_rate = (current_success_rate * steps_count + (1 if is_success else 0.5)) / (steps_count + 1)
            self.robot_chosen_action_prob[robot_id] = (steps_count + 1, new_success_rate)

    # In general, we want to consider both a greedy solution, oriented towards short term gains
    # and a more strategical solution, aimed at satiating all plants and getting the goal_reward.
    # we will compare the performance expected from either method and determine which is the one
    # GREEDY SOLUTION:
    #   in the gredy solution, we compare between two different types of paths:
    #       - a path in which we go directly to a plant
    #       - a path in which we head to a tap, load water and only then head toward the plant
    #       * it should be noted, that the latter path type turns in due time into a path of the first type,
    #           as soon as it has completed loading
    #   we infer the kind of path we're working with from the {tap} variable, where it being {None} indicates
    #   the first type of path, and we deduce that it is a path of the second type if the circumstances are reversed.
    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        if not self.is_in_run:
            self.is_in_run = True
            self.current_run_mean_reward = 0
            # The per-run-statistics are only relevant in terms of yield management, which is in other words the study of when should one reset.
            # for this reason, I pertain the "RESET" move to one of the moves associated with each run, and hence effecting its final {best_run_mean_reward_per_step}
            self.current_run_length = 1 # for some reason this works better when initialised to 0, even though it does not account for the RESET action

        if self.is_in_run:
            self.current_run_length += 1

        self.check_if_last_action_succeeded(state)
        self.previous_state = state
        (robots, plants, taps, total_water_need) = state
        non_empty_taps = [tap for tap in taps if tap[1] > 0]
        non_empty_plants = [plant for plant in plants if plant[1] > 0]
        total_robot_load = sum(robot[2] for robot in robots)
        if total_robot_load == 0 and len(non_empty_taps) == 0:
            self.is_in_run = False
            self.last_action = "RESET"
            return "RESET"

        # checking if any movement action "opens up the board" and adds better greedy solutions
        best_lookahead_path_data = self.find_lookahead_best_move(robots, non_empty_plants, non_empty_taps)

        # checking if there is a sub-optimal path to water a plant leads to a better overall result
        # than the greedy solution. This is very effective in problem_pdf2 for example
        best_planned_lookahead_path_data = self.get_best_planned_lookahead(robots, non_empty_plants, non_empty_taps)

        # getting best greedy path (we'll only execute the first action in the path every time,
        # but the path should remain consistent across different choose_next_action invocations)
        best_greedy_path_data = self.find_greedy_best_robot_plant(robots, non_empty_plants, non_empty_taps)

        if best_planned_lookahead_path_data is not None and best_greedy_path_data is not None:
            (robot, plant, tap_cords, greedy_mean_reward_per_step, _, __, ___) = best_greedy_path_data
            (planned_mean_reward_per_step, planned_path_data) = best_planned_lookahead_path_data
            if greedy_mean_reward_per_step < planned_mean_reward_per_step:
                best_greedy_path_data = planned_path_data
        elif best_planned_lookahead_path_data is not None and best_greedy_path_data is None:
            (planned_mean_reward_per_step, planned_path_data) = best_planned_lookahead_path_data
            best_greedy_path_data = planned_path_data

        if best_greedy_path_data is None and best_lookahead_path_data is None and best_planned_lookahead_path_data is None:
            self.is_in_run = False
            self.last_action = "RESET"
            return "RESET"

        max_mean_reward_per_step = -float('inf')
        lookahead_action = None

        if best_lookahead_path_data is not None:
            (mean_reward_per_step, action) = best_lookahead_path_data
            max_mean_reward_per_step = mean_reward_per_step
            lookahead_action = action
            if best_greedy_path_data is None: # this is an awful way to write this logic
                if (max_mean_reward_per_step < self.best_run_mean_reward_per_step
                    and self.best_run_length <= self.original_game.get_max_steps() - self.original_game.get_current_steps()):
                    self.is_in_run = False
                    self.last_action = "RESET"
                    return "RESET"
                else:
                    self.last_action = lookahead_action
                    return lookahead_action

        (robot, plant, tap_cords, greedy_mean_reward_per_step, _, __, ___) = best_greedy_path_data
        should_use_lookahead = True
        if max_mean_reward_per_step < greedy_mean_reward_per_step:
            max_mean_reward_per_step = greedy_mean_reward_per_step
            should_use_lookahead = False

        if (max_mean_reward_per_step < self.best_run_mean_reward_per_step
            and self.best_run_length <= self.original_game.get_max_steps() - self.original_game.get_current_steps()):
            self.is_in_run = False
            self.last_action = "RESET"
            return "RESET"

        if should_use_lookahead and lookahead_action is not None:
            self.last_action = lookahead_action
            return lookahead_action

        (robot_id, robot_cords, load) = robot
        capacities = self.original_game.get_capacities()
        capacity = capacities[robot_id]
        remaining_capacity = capacity - load
        success_rate = self.robot_chosen_action_prob[robot_id][1]
        (plant_cords, water_needed) = plant

        other_robot_cords = set(_robot[1] for _robot in robots if _robot[0] != robot_id)
        self.update_bfs_distances(other_robot_cords)

        # the order here matters a lot.
        # it is possible that the robot is directly standing on the plant it seeks to satiate
        # but needs to head over to the tap in order to get some water and then satiate the plant fully
        # tbh, if the robot has any load it should POUR first.

        if robot_cords == plant_cords:
            if load > 0:
                self.current_run_mean_reward += self.get_mean_reward_for_pour(robot_id, robot_cords)
                self.update_best_run()
                self.last_action = f"POUR({robot_id})"
                return f"POUR({robot_id})"

        if tap_cords is not None:
            if robot_cords == tap_cords:
                self.last_action = f"LOAD({robot_id})"
                return f"LOAD({robot_id})"
            action_name = self.get_movement_action_in_path(robot_cords, tap_cords)
            self.last_action = f"{action_name}({robot_id})"
            return f"{action_name}({robot_id})"
        if len(robots) == 1 and robot_cords in [non_empty_tap_cords for (non_empty_tap_cords, _) in non_empty_taps] and remaining_capacity > 0: # should add something about the horizon here.
            if load < water_needed / success_rate:
                self.last_action = f"LOAD({robot_id})"
                return f"LOAD({robot_id})"


        # we need to get to the plant
        action_name = self.get_movement_action_in_path(robot_cords, plant_cords)
        self.last_action = f"{action_name}({robot_id})"
        return f"{action_name}({robot_id})"

    def get_mean_reward_for_pour(self, robot_id, robot_cords):
        plant_mean_reward_per_water_unit = self.plant_rewards[robot_cords]
        success_rate = self.robot_chosen_action_prob[robot_id][1]
        return success_rate * plant_mean_reward_per_water_unit

    def update_best_run(self):
        if self.current_run_length == 0 or self.current_run_mean_reward == 0:
            return

        current_run_mean_reward_per_step = self.current_run_mean_reward / self.current_run_length
        remaining_horizon = self.original_game.get_max_steps() - self.original_game.get_current_steps()

        is_best_run_too_long_for_horizon = remaining_horizon < self.best_run_length
        is_current_run_reward_higher = current_run_mean_reward_per_step > self.best_run_mean_reward_per_step
        is_possible_to_repeat_current_run = self.current_run_length <= remaining_horizon
        if ((is_current_run_reward_higher or is_best_run_too_long_for_horizon) and is_possible_to_repeat_current_run):
            self.best_run_length = self.current_run_length
            self.best_run_mean_reward_per_step = current_run_mean_reward_per_step

    def get_legal_movement_actions(self, robot: Robot, robots: list[Robot]):
        possible_moves = []

        directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
        robot_id, (r, c), load = robot
        for (dr, dc), action_name in directions:
            destination = (r + dr, c + dc)
            if (all(cords != destination for (_, cords, __) in robots) and
                destination not in self.original_game.walls and
                0 <= r + dr < self.original_game.rows and
                0 <= c + dc < self.original_game.cols):
                possible_moves.append(((r + dr, c + dc), action_name))
        return possible_moves

    def calc_mean_steps(self, src: Cords, dst: Cords, success_rate: float):
        (x_1, x_2) = src
        (y_1, y_2) = dst
       
        distance = self.bfs_distances.get((dst, src), float('inf'))
        # corridor situation
        # In case of failure, we have 4 options, moving UP, DOWN, RIGHT, LEFT and staying ***MINUS*** the wanted operation.
        # In the corridor situation 3/4 directions drift us away, and staying in place is neutral.
        # So the expectation of steps on failure is:
        # 3 * 0.25 * (-1) + 0.25 * 0 = -0.75
        # Now the total expectation of a step (including both failure and success), is:
        # success_rate * 1 - 0.75 * (1 - success_rate) = 1.75 * success_rate - 0.75
        # and from here we would need distance / (1.75 * success_rate - 0.75) steps to reach dst.
        # this is a direct deduction from Wald's theorem: https://en.wikipedia.org/wiki/Wald%27s_equation

        if (x_1 == y_1 or x_2 == y_2):
            return distance / (1.75 * success_rate - 0.75)

        # non-corridor situation, we need to circle around a block
        # We once more have 4 options (on failure), moving in one of the three remaining directions, and staying in place.
        # The difference is that in this case, one of the three directions advances us,
        # two of them harm us, and staying in place is neutral.
        # the above yields the following EXPECTATION of advancement per failure:
        #   E = 2 * 0.25 * (-1) + 1 * 0.25 * 1 + 0 * 0.25 = -0.25
        # this means that the EXPECTED advancement, on failure is -0.25.
        # from here, the mean advancement in general is:
        # E = success_rate * 1 + (1 - success_rate) * (-1) * 0.25 = 1.25 * success_rate - 0.25
        # and for this reason, using Wald's Theorem or common sense, we find that:
        # E[steps_required] = distance / (1.25 * success_rate - 0.25) 
        return distance / (1.25 * success_rate - 0.25)


    # the only way in which this BFS function is different than normal BFS
    # is that we favour paths that are not straihgt, but instead include ZigZags.
    # the reason due to which we favour such paths, is that failing to step in
    # the correct direction in such paths, can lead to an accidental movement which advances
    # us in the path, even though we did not directly wish to reach it.
    # for example, in a map without obstacles, moving from (0,0) towards (3,3) by going to
    # (1,0) is just the same as going to (0,1), and so we'd favour paths that have the most ZigZag-ed-ness
    def bfs(self, cords: Cords, walls: set[Cords]):
        """Expects the coordinates of a plant / tap, and in return calculates the minimal distance from the entity to any point lying on the map"""
        if cords in walls:
            return
        visited_nodes: set[Cords] = set() 
        queue = []
        heapq.heappush(queue, (0, 0, cords, (0, 0)))

        self.bfs_distances[(cords, cords)]  = 0
        self.bfs_paths[(cords, cords)]      = {cords}

        (height, width) = self.original_game.rows, self.original_game.cols
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and (i,j) not in walls)

        possible_actions = [(0,-1), (0,1), (-1, 0), (1,0)]

        while queue:
            dist, penalty, old_point, (last_dr, last_dc) = heapq.heappop(queue)
            (node_r, node_c) = old_point

            if old_point in visited_nodes:
                continue

            visited_nodes.add(old_point)

            for (d_r, d_c) in possible_actions:
                new_point = (node_r + d_r, node_c + d_c)
                if (is_position_legal(node_r + d_r, node_c + d_c)
                    and not (node_r + d_r, node_c + d_c) in visited_nodes):

                    new_penalty = penalty + (1 if last_dr == d_r or last_dc == d_c else 0)
                    new_dist = dist + 1

                    self.bfs_distances[(cords, new_point)] = self.bfs_distances[(cords, old_point)] + 1
                    self.bfs_paths[(cords, new_point)] = self.bfs_paths[(cords, old_point)].union({ new_point })

                    heapq.heappush(queue, (new_dist, new_penalty, new_point, (d_r, d_c)))

    def update_bfs_distances(self, extra_walls: set[Cords]):
        total_walls = self.original_game.walls.union(extra_walls)
        self.bfs_distances = {}
        self.bfs_paths = {}
        for cords in self.original_game.taps:
            self.bfs(cords, total_walls)
        for cords in self.original_game.plants:
            self.bfs(cords, total_walls)

# We are considering two possible situations:
#   - Going to a plant directly, and watering it
#   - Going to a tap, then from there going to a plant and watering it.
# The second situation is a bit more complicated, so here is how I model it:
# for every robot and plant:
# go over every tap, and calculate the value of loading water units in the following amount:
# M := min(tap_available_water, (plant_water_needed / success_rate) - current_robot_load, robot_remaining_capacity)
# * plant_water_needed / success_rate is the EXPECTED amount of water needed to satiate the plant with this particular robot
# now, get the amount of steps needed to get to the tap, and from the tap to the plant using calc_mean_step.
# get the mean of the total number of turns as being: (M / success_rate) loading operations (when failure occurs, we don't load anything but just waste a turn) + the mean amount of steps required to get to the tap and from there to the plant + M (the amount of times we POUR on the plant)
# now, get the mean reward for this option as: M * success_rate * plant_reward_per_water_unit
# at last, divide the mean reward for this option by the mean total amount of turns  to get the mean per step reward
# from this we find the best greedy option for going to a tap then to a plant, we will later compare it with going directly to the plant.
#
# Returning to the second case, I simply note the mean amount of steps required to reach the plant, and the amount of load the robot can pour on the plant (depends on the amount of water it carries, and on the water needed by the plant)
# then, I calculate the reward per step just as I had previously done.

    def eval_robot_plant(self, robot: Robot, plant: Plant, taps: list[Tap], goal_reward_per_water_unit: float, preceding_steps: int, horizon: int):
        # I set up the BFS before calling eval_robot_plant

        # Getting remaining horizon
        remaining_horizon = horizon if horizon >= 0 else self.original_game.get_max_steps() - self.original_game.get_current_steps() - preceding_steps

        # Getting Robot info
        capacities = self.original_game.get_capacities()
        (robot_id, robot_cords, load) = robot
        capacity = capacities[robot_id]
        remaining_capacity = capacity - load
        success_rate = self.robot_chosen_action_prob[robot_id][1]

        # Getting plant info
        (plant_cords, water_needed) = plant
        mean_water_needed_to_satiate_plant = np.ceil((water_needed / success_rate))
        mean_water_missing_to_satiate = max(0, mean_water_needed_to_satiate_plant - load)
        plant_mean_reward_per_water_unit = self.plant_rewards[plant_cords]
        plant_mean_reward_per_water_unit += goal_reward_per_water_unit
        max_mean_reward_per_step_for_path = -float('inf')
        # Path going through a tap
        max_tap_cords = None
        max_load_used = 0
        max_water_loaded = 0
        max_step_count = 0

        if remaining_capacity > 0:
            for tap_cords, tap_available_water in taps:
                # M := the amount of water we can LOAD:
                #   - can't be more than the tap has
                #   - can't be more than the plant needs (?) perhaps this is wrong
                #   - can't be more than the robot can carry
                #   - can't be more than the horizon allows
                #           this one is quite intricate, what does it mean for the horizon to allow something?
                #           the idea is that we are bound to take {mean_steps_to_tap_then_plant} steps no matter what
                #           this leaves us with {remaining_horizon} - {mean_steps_to_tap_then_plant} steps
                #           we ideally want to pour all the {load} to pour, before loading anything from a tap
                #           so we are left with: {remaining_horizon} - {mean_steps_to_tap_then_plant} - {load} steps
                #           these steps must be divided into two groups:
                #               - LOAD steps
                #               - POUR steps
                #           we'd hate to use extra LOAD steps, then not use POUR on everything
                #           the mean load we get per LOAD operation is success_rate.
                #           after {K} LOAD operations, we're left with {K} * {success_rate} amount of {load}
                #           and so we want to ascertain that the {remaining_horizon} after the extra LOAD operations
                #           does not fall from {K} * {success_rate}, meaning
                #           {K} * {success_rate} < {remaining_horizon} - {mean_steps_to_tap_then_plant} - {load} - {K}
                #       =>  {K} * {1 + success_rate} < {remaining_horizon} - {mean_steps_to_tap_then_plant} - {load}
                #       =>  {K} < ({remaining_horizon} - {mean_steps_to_tap_then_plant} - {load}) / (1 + {success_rate})
                #           and from here:
                #       additionally, {K} is the amount of load operations, {M}, is the actual load. The relationship between them is of course {M} = {K} * success_rate
                #   - can't be more than (({remaining_horizon} - {mean_steps_to_tap_then_plant} - {load}) / (1 + {success_rate})) * success_rate

                mean_steps_to_tap_then_plant = self.calc_mean_steps(robot_cords, tap_cords, success_rate) + self.calc_mean_steps(tap_cords, plant_cords, success_rate)
                if mean_steps_to_tap_then_plant == float('inf'):
                    continue
                horizon_load_steps_constraint = (remaining_horizon - mean_steps_to_tap_then_plant - load) / (1 + success_rate)
                horizon_actual_load_constraint = horizon_load_steps_constraint * success_rate
                M = min(tap_available_water, mean_water_missing_to_satiate, remaining_capacity, horizon_actual_load_constraint)
                if M <= 0: # a case where we don't need / cannot take additional water is handlded below, where we go directly to the plant without visiting any taps along the way
                    continue
                mean_poured_units = min(M + load, mean_water_needed_to_satiate_plant) # including SPILL
                mean_successful_pours = mean_poured_units * success_rate
                #if mean_successful_pours >= water_needed or capacity < mean_water_needed_to_satiate_plant:
                #    plant_mean_reward_per_water_unit += goal_reward_per_water_unit
                mean_steps_for_path = (M / success_rate) + mean_steps_to_tap_then_plant + mean_poured_units
                mean_reward_for_path = mean_successful_pours * plant_mean_reward_per_water_unit
                mean_reward_per_step_for_path = mean_reward_for_path / (mean_steps_for_path + preceding_steps)
                if mean_reward_per_step_for_path > max_mean_reward_per_step_for_path:
                    max_tap_cords = tap_cords
                    max_mean_reward_per_step_for_path = mean_reward_per_step_for_path
                    max_load_used = min(mean_poured_units, load)
                    max_water_loaded = M
                    max_step_count = mean_steps_for_path + preceding_steps

        # Direct Path
        if load > 0:
            mean_steps_to_plant = self.calc_mean_steps(robot_cords, plant_cords, success_rate)
            if mean_steps_to_plant == float('inf'):
                return (max_mean_reward_per_step_for_path, max_tap_cords, max_load_used, max_water_loaded, max_step_count)

            mean_poured_units = min(load, mean_water_needed_to_satiate_plant, remaining_horizon - mean_steps_to_plant) # including SPILL, and accounting for the remaining horizon.
            mean_successful_pours = mean_poured_units * success_rate
            #if mean_successful_pours >= water_needed or capacity < mean_water_needed_to_satiate_plant:
            #    plant_mean_reward_per_water_unit += goal_reward_per_water_unit
            mean_reward_for_path = mean_successful_pours * plant_mean_reward_per_water_unit
            mean_steps_for_path = mean_steps_to_plant + mean_poured_units
            mean_reward_per_step_for_path = mean_reward_for_path / (mean_steps_for_path + preceding_steps)
            if mean_reward_per_step_for_path > max_mean_reward_per_step_for_path:
                max_tap_cords = None # meaning we don't need to go through a tap
                max_mean_reward_per_step_for_path = mean_reward_per_step_for_path
                max_load_used = mean_poured_units
                max_water_loaded = 0
                max_step_count = mean_steps_for_path + preceding_steps

        return (max_mean_reward_per_step_for_path, max_tap_cords, max_load_used, max_water_loaded, max_step_count)

    # Iterates over every robot & plant pair in order to find the pair with the highest {mean_reward_per_step_for_path},
    # then take the first action in the selected path, and repeat the first step (finding the optimal path)
    # this path will likely remain prevalent (in terms of reward per step rate)
    # (the reward per step probably increased if we took the right step), so we will probably follow it even if we allow
    # shifting to another plan.
    # additionally, we incorporate the {goal_reward} into our evaluation, by adding {goal_reward} / {total_water_units_missing}
    # to every plant's reward, if we would not have done this, the algorithm would often favour to satiate the "best" plant,
    # then simply reset the problem and satiate it once more repeatedly, until the horizon is depleted.

    def find_greedy_best_robot_plant(self, robots: list[Robot], plants: list[Plant], taps: list[Tap], preceding_steps: int =0, horizon: int = -1):
        best_success_rate = max(self.robot_chosen_action_prob[robot[0]][1] for robot in robots)
        total_water_units_missing = sum(plant[1] for plant in plants)
        min_mean_remaining_pours = total_water_units_missing / best_success_rate
        total_water_units_available_in_taps = sum(tap[1] for tap in taps)
        total_water_units_available_in_robots = sum(robot[2] for robot in robots)
        if total_water_units_available_in_taps + total_water_units_available_in_robots < 0.75 * min_mean_remaining_pours:
            goal_reward_per_water_unit = 0
        # this also lowers the final reward, but I do think it to be correct, I also think I should divide it by  best_success_rate
        #elif self.original_game.get_max_steps() - self.original_game.get_current_steps() <= (2 * total_water_units_missing - total_water_units_available_in_robots):
        #    goal_reward_per_water_unit = 0
        else:
            goal_reward_per_water_unit = self.original_game._goal_reward / total_water_units_missing

        max_mean_reward_per_step_for_path = -float('inf')
        max_tap_cords = None
        max_robot = None
        max_plant = None
        max_load_used = 0
        max_water_loaded = 0
        max_step_count = 0

        for robot in robots:
            (robot_id, robot_cords, load) = robot

            success_rate = self.robot_chosen_action_prob[robot_id][1]
            if len(robots) == 2 and best_success_rate - success_rate > 0.09: # this is fucked up, I will admit.
                continue
            other_robots = list(_robot for _robot in robots if _robot[0] != robot_id)
            other_robot_cords = set((robot[1] for robot in other_robots))
            self.update_bfs_distances(other_robot_cords)

            for plant in plants:
                (mean_reward_per_step_for_path, tap_cords, load_used, water_loaded, step_count) = self.eval_robot_plant(robot, plant, taps, goal_reward_per_water_unit, preceding_steps, horizon=horizon)
                if max_mean_reward_per_step_for_path < mean_reward_per_step_for_path:
                    max_mean_reward_per_step_for_path = mean_reward_per_step_for_path
                    max_tap_cords = tap_cords
                    max_robot = robot
                    max_plant = plant
                    max_load_used = load_used
                    max_water_loaded = water_loaded
                    max_step_count = step_count
        if max_mean_reward_per_step_for_path <= 0 or max_plant is None or max_robot is None:
            # get random action
            return None
        return (max_robot, max_plant, max_tap_cords, max_mean_reward_per_step_for_path, max_load_used, max_water_loaded, max_step_count)

    def get_movement_action_in_path(self, src: Cords, dest: Cords):
        current_distance = self.bfs_distances.get((dest, src), None)
        if current_distance is None:
            raise Exception(f"Path from {src} to {dest} does not exist in self.bfs_distances")

        directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
        (src_r, src_c) = src
        good_next_steps = []
        for (dr, dc), action_name in directions:
            potential_next_step = (src_r + dr, src_c + dc)
            next_distance = self.bfs_distances.get((dest, potential_next_step), float('inf'))

            if next_distance < current_distance:
                good_next_steps.append((potential_next_step, action_name))

        (dest_r, dest_c) = dest
        for (next_step, action) in good_next_steps:
            (n_r, n_c) = next_step
            if max(abs(n_r - dest_r), abs(n_c - dest_c)) < max(abs(src_r - dest_r), abs(src_c - dest_c)):
                return action
        (_, first_good_action) = good_next_steps[0]
        return first_good_action

    def find_lookahead_best_move(self, robots: list[Robot], plants: list[Plant], taps: list[Tap]):
        max_mean_reward_per_step = -float('inf')
        max_action = None

        staying_in_place_path_data = self.find_greedy_best_robot_plant(robots, plants, taps, preceding_steps=1)
        if staying_in_place_path_data is None:
            return None

        (_, __, ___, staying_in_place_mean_reward_per_step, ____, _____, ________) = staying_in_place_path_data
        for robot in robots:
            robot_expected_move_values = {}
            robot_id, (r, c), load = robot
            success_rate = self.robot_chosen_action_prob[robot_id][1]
            legal_moves = self.get_legal_movement_actions(robot, robots)
            for robot_new_cords, action_name in legal_moves:
                new_robots = []
                for robot in robots:
                    if robot[0] == robot_id:
                        new_robots.append((robot_id, robot_new_cords, robot[2]))
                    else:
                        new_robots.append(robot)
                best_path_data = self.find_greedy_best_robot_plant(new_robots, plants, taps, preceding_steps=1)
                if best_path_data is not None:
                    (robot, plant, tap_cords, mean_reward_per_step, ______, _______, ________) = best_path_data
                    robot_expected_move_values[action_name] = mean_reward_per_step
                else:
                    # this situation indicates that a reset would've been prevalent
                    if (max_mean_reward_per_step < self.best_run_mean_reward_per_step
                        and self.best_run_length <= self.original_game.get_max_steps() - self.original_game.get_current_steps()):
                        robot_expected_move_values[action_name] = self.best_run_mean_reward_per_step
                    else:
                        pass
                    #return None
                    #raise Exception("This scenario requires a reset. If this exception will pop in the tests then I'll do it, but I wnat to check if it pops first")

            for action_name, expected_value_of_end_state in robot_expected_move_values.items():
                other_moves_cumulative_value = sum(expected_value_other_end_state for (other_action, expected_value_other_end_state) in robot_expected_move_values.items() if other_action != action_name)
                other_moves_cumulative_value_with_staying = other_moves_cumulative_value + staying_in_place_mean_reward_per_step
                other_moves_mean_value = (1 / len(robot_expected_move_values)) * other_moves_cumulative_value_with_staying
                expected_action_value_per_step = success_rate * expected_value_of_end_state + (1 - success_rate) * other_moves_mean_value
                if max_mean_reward_per_step < expected_action_value_per_step:
                    max_mean_reward_per_step = expected_action_value_per_step
                    max_action = f"{action_name}({robot_id})"

        if max_mean_reward_per_step == -float('inf') or max_action == None:
            return None
        return (max_mean_reward_per_step, max_action)

                # trying the next depth. This is quite a silly implementation.
                # it does not work well, because I did not account for the robot's success_rate, and so it
                # can generate a plan that's pretty unlikely to really be happen 


                # because for every robot, we consider every possible move anyways, and calculate its greedy value
                # then for every robot movement, I can calculate the movements's greedy value using the following formula:
                #   V = {success_rate} * post_move_greedy_value + (1 - {success_rate}) / 5 * greedy_value_after_going_left + (1 - {success_rate}) / 5 * greedy_value_after_going_down + (1 - {success_rate}) / 5 * greedy_value_after_going_right + (1 - {success_rate}) / 5 * greedy_value_after_going_up + (1 - {success_rate}) / 5 * greedy_value_staying_in_place

                #for new_robot_id, new_robot_new_cords, _ in self.get_legal_movement_actions(new_robots):
                #    new_new_robots = []
                #    for robot in new_robots:
                #        if robot[0] == new_robot_id:
                #            new_new_robots.append((new_robot_id, new_robot_new_cords, robot[2]))
                #        else:
                #            new_new_robots.append(robot)
                #    best_path_data = self.find_greedy_best_robot_plant(new_new_robots, plants, taps, preceding_steps=2)
                #    if best_path_data is not None:
                #        (__, ___, ____, mean_reward_per_step) = best_path_data
                #        if max_mean_reward_per_step < mean_reward_per_step:
                #            max_mean_reward_per_step = mean_reward_per_step
                #            max_action = f"{action_name}({robot_id})" # the action in which we are interested is the first action in the sequence

        #if max_mean_reward_per_step == -float('inf') or max_action == None:
        #    return None
        #return (max_mean_reward_per_step, max_action)


    # the issue;
    # in problem_pdf2, the optimal solution is along the lines of satiating both plants using the higher
    # probability robot, commencing with the plant on the bottom left
    # the reason due to which it does not happen (as of the moment before writing the following function)
    # is that greedily, the top-right plant provides a higher mean reward per step, and hence is going to
    # be satiated first.
    # what we can do, is use a lookahead based strategy, to the mean reward per step, in a plan where
    # we first satiate a non-optimal plant, and then head over to the optimal plant.
    # this strategy feels a little bafoonish, but it aims specifically at scenarios as those in the pdf problems
    # and I will venture to allow it in this instance as it would appear that my current runtime is so low
    # that it most genuinely wouldn't harm to add such remarkably pulverulent calculation
    def get_best_planned_lookahead(self, robots: list[Robot], plants: list[Plant], taps: list[Tap]):
        if len(plants) < 2:
            return None
        remaining_horizon = self.original_game.get_max_steps() - self.original_game.get_current_steps()

        max_mean_reward_per_step = -float('inf')
        max_path_data = None

        for plant in plants:
            new_plants = [plant]
            best_greedy_path_data = self.find_greedy_best_robot_plant(robots, new_plants, taps)
            if best_greedy_path_data is None:
                continue
            (robot, plant, tap_cords, greedy_mean_reward_per_step, load_used, water_loaded, step_count) = best_greedy_path_data
            success_rate = self.robot_chosen_action_prob[robot[0]][1]
            new_robots = []
            for r in robots:
                if r[0] == robot[0]:
                    new_robots.append((robot[0], plant[0], robot[2] - load_used)) # not sure that the current load's calculation is correct...
                else:
                    new_robots.append(r)
            new_plants = []
            for p in plants:
                if p[0] == plant[0]:
                    new_water_needed = p[1] - success_rate * (load_used + water_loaded)
                    if new_water_needed <= 0:
                        continue
                    new_plants.append((p[0], new_water_needed))
                else:
                    new_plants.append(p)
            new_taps = taps
            if tap_cords:
                new_taps = []
                for tap in taps:
                    if tap[0] == tap_cords:
                        new_taps.append((tap[0], tap[1] - water_loaded))
                    else:
                        new_taps.append(tap)
            new_horizon = remaining_horizon - step_count
            if new_horizon <= 0:
                continue
            second_best_greedy_path_data = self.find_greedy_best_robot_plant(new_robots, new_plants, new_taps, horizon=new_horizon)
            if second_best_greedy_path_data is None:
                continue
            (_, __, ___, second_greedy_mean_reward_per_step, ____, _____, second_step_count) = second_best_greedy_path_data
            if step_count + second_step_count > 0.7 * remaining_horizon: # 0.7 works well but there is no particular thought behind it
                continue
            combined_path_total_reward = greedy_mean_reward_per_step * step_count + second_greedy_mean_reward_per_step * second_step_count
            combined_mean_reward_per_step = combined_path_total_reward / (step_count + second_step_count)
            if combined_mean_reward_per_step > max_mean_reward_per_step:
                max_mean_reward_per_step = combined_mean_reward_per_step
                max_path_data = best_greedy_path_data

        if max_path_data is None or max_mean_reward_per_step == -float('inf'):
            return None
        return (max_mean_reward_per_step, max_path_data)
