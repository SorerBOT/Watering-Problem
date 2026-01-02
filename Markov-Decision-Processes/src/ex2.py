from re import L
import ext_plant
import numpy as np
from collections import deque

id = ["000000000"]

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
        (robots, plants, taps, total_water_need) = state
        best_path_data = self.find_greedy_best_robot_plant(robots, plants, taps)
        if best_path_data is None:
            return "RESET"
        (robot, plant, tap_cords) = best_path_data
        (robot_id, robot_cords, load) = robot
        (plant_cords, water_needed) = plant

        other_robot_cords = set(_robot[1] for _robot in robots if _robot[0] != robot_id)
        self.update_bfs_distances(other_robot_cords)

        # the order here matters a lot.
        # it is possible that the robot is directly standing on the plant it seeks to satiate
        # but needs to head over to the tap in order to get some water and then satiate the plant fully
        # tbh, if the robot has any load it should POUR first.

        if robot_cords == plant_cords:
            if load > 0:
                return f"POUR({robot_id})"

        if tap_cords is not None:
            if robot_cords == tap_cords:
                return f"LOAD({robot_id})"
            action_name = self.get_movement_action_in_path(robot_cords, tap_cords)
            return f"{action_name}({robot_id})"

        # we need to get to the plant
        action_name = self.get_movement_action_in_path(robot_cords, plant_cords)
        return f"{action_name}({robot_id})"

    #def choose_next_action(self, state):
    #    """ Choose the next action given a state."""
    #    possible_moves = []

    #    (robots, plants, taps, total_water_need) = state
    #    capacities = self.original_game.get_capacities()

    #    directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
    #    for robot_id, (r, c), load in robots:
    #        remaining_capacity = capacities[robot_id] - load
    #        for (dr, dc), action_name in directions:
    #            destination = (r + dr, c + dc)
    #            if (all(cords != destination for (_, cords, __) in robots) and
    #                destination not in self.original_game.walls and
    #                0 <= r + dr < self.original_game.rows and
    #                0 <= c + dc < self.original_game.cols):
    #                possible_moves.append(f"{action_name}({robot_id})")
    #        if load > 0:
    #            plant_in_current_location = next(((pos, need) for (pos, need) in plants if (r, c) == pos and need > 0), None)
    #            if plant_in_current_location is not None:
    #                possible_moves.append(f"POUR({robot_id})")
    #        if remaining_capacity > 0:
    #            tap_in_current_location = next(((pos, available_water) for (pos, available_water) in taps if (r, c) == pos and available_water > 0), None)
    #            if tap_in_current_location is not None:
    #                possible_moves.append(f"LOAD({robot_id})")
    #    return np.random.choice(np.array(possible_moves))

    def calc_mean_steps(self, src: Cords, dst: Cords, success_rate: float):
        (x_1, x_2) = src
        (y_1, y_2) = dst;
       
        distance = self.bfs_distances.get((dst, src), float('inf'))
        # corrisor situation
        # In case of failure, we have 5 options, moving UP, DOWN, RIGHT, LEFT and staying in place.
        # In the corridor situation 3/4 directions drift us away, 1 direction advances us
        # and staying in place is neutral. So the expectation of steps on failure is:
        # 3 * 0.2 * (-1) + 0.2 + 0.2 * 0 = -0.4
        # Now the total expectation of a step (including both failure and success), is:
        # success_rate * 1 - 0.4 *(1 - success_rate) = 1.4 * success_rate - 0.4
        # and from here we would need distance / (1.4 * success_rate - 0.4) steps to reach dst.
        # this is a direct deduction from Wald's theorem: https://en.wikipedia.org/wiki/Wald%27s_equation
        if (x_1 == y_1 or x_2 == y_2):
            return distance / (1.4 * success_rate - 0.4)

        # non-corridor situation, we need to circle around a block
        # We once more have 5 options, moving in one of the four directions, and staying in place.
        # the difference is that in this case, two of the four directions advance us, and 2 harm us, while staying in place is neutral.
        # the above yields the following EXPECTATION of advancement per failure:
        #   E = 2 * 0.2 * (-1) + 2 * 0.2 * 1 + 0.2 * 0 = 0
        # this means that the EXPECTED advancement, on failure is 0.
        # from here, the mean advancement in general is:
        # E = success_rate * 1 + (1 - success_rate) * 0 = success_rate
        # and for this reason, using Wald's Theorem or common sense, we find that:
        # E[steps_required] = distance / success_rate
        return distance / success_rate
        

    def bfs(self, cords: Cords, walls: set[Cords]): # accepts a list, as we may have several src points (all plants to whatever taps, but single tap to whatever point)
        """Expects the coordinates of a plant / tap, and in return calculates the minimal distance from the entity to any point lying on the map"""
        visited_nodes: set[tuple[int, int]] = set({ cords }) 
        queue = deque()

        queue.append(cords)

        (height, width) = self.original_game.rows, self.original_game.cols
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and (i,j) not in walls)

        self.bfs_distances[(cords, cords)]  = 0
        self.bfs_paths[(cords, cords)]      = {cords}
        possible_actions = [(0,-1), (0,1), (-1, 0), (1,0)]

        while (len(queue) > 0):
            (node_i, node_j) = old_point = queue.pop()
            for (d_i, d_j) in possible_actions:
                new_point = (node_i + d_i, node_j + d_j)
                if (is_position_legal(node_i + d_i, node_j + d_j)
                    and not (node_i + d_i, node_j + d_j) in visited_nodes):

                    queue.append(new_point)
                    self.bfs_distances[(cords, new_point)] = self.bfs_distances[(cords, old_point)] + 1
                    self.bfs_paths[(cords, new_point)] = self.bfs_paths[(cords, old_point)].union({ new_point })
                    visited_nodes.add((node_i + d_i, node_j + d_j))

    def update_bfs_distances(self, extra_walls: set[Cords]):
        total_walls = self.original_game.walls.union(extra_walls)
        for cords in self.original_game.taps:
            self.bfs(cords, self.original_game.walls.union(total_walls))
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

    def eval_robot_plant(self, robot: Robot, plant: Plant, taps: list[Tap], other_robots: list[Robot]):
        # Setting up BFS distances
        other_robot_cords = set((robot[1] for robot in other_robots))
        self.update_bfs_distances(other_robot_cords)

        # Getting remaining horizon
        remaining_horizon = self.original_game.get_max_steps() - self.original_game.get_current_steps()

        # Getting Robot info
        capacities = self.original_game.get_capacities()
        (robot_id, robot_cords, load) = robot
        capacity = capacities[robot_id]
        remaining_capacity = capacity - load
        success_rate = self.original_game._robot_chosen_action_prob[robot_id]

        # Getting plant info
        (plant_cords, water_needed) = plant
        mean_water_needed_to_satiate_plant = np.ceil((water_needed / success_rate))
        mean_water_missing_to_satiate = max(0, mean_water_needed_to_satiate_plant - load)
        plant_rewards = self.original_game._plants_reward[plant_cords]
        plant_mean_reward_per_water_unit = sum(plant_rewards) / len(plant_rewards)

        max_mean_reward_per_step_for_path = -float('inf')
        # Path going through a tap
        max_tap_cords = None
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

            mean_steps_to_tap_then_plant = self.calc_mean_steps(tap_cords, robot_cords, success_rate) + self.calc_mean_steps(tap_cords, plant_cords, success_rate)
            horizon_load_steps_constraint = (remaining_horizon - mean_steps_to_tap_then_plant - load) / (1 + success_rate)
            horizon_actual_load_constraint = horizon_load_steps_constraint * success_rate
            M = min(tap_available_water, mean_water_missing_to_satiate, remaining_capacity, horizon_actual_load_constraint)
            if M <= 0: # a case where we don't need / cannot take additional water is handlded below, where we go directly to the plant without visiting any taps along the way
                continue
            mean_poured_units = min(M + load, mean_water_needed_to_satiate_plant) # including SPILL
            mean_steps_for_path = (M / success_rate) + mean_steps_to_tap_then_plant + mean_poured_units
            mean_reward_for_path = mean_poured_units * success_rate * plant_mean_reward_per_water_unit
            mean_reward_per_step_for_path = mean_reward_for_path / mean_steps_for_path
            if mean_reward_per_step_for_path > max_mean_reward_per_step_for_path:
                max_tap_cords = tap_cords
                max_mean_reward_per_step_for_path = mean_reward_per_step_for_path

        # Direct Path
        if load > 0:
            mean_steps_to_plant = self.calc_mean_steps(plant_cords, robot_cords, success_rate)
            mean_poured_units = min(load, mean_water_needed_to_satiate_plant, remaining_horizon - mean_steps_to_plant) # including SPILL, and accounting for the remaining horizon.
            mean_steps_for_path = mean_steps_to_plant + mean_poured_units
            mean_reward_for_path = mean_poured_units * success_rate * plant_mean_reward_per_water_unit
            mean_reward_per_step_for_path = mean_reward_for_path / mean_steps_for_path
            if mean_reward_per_step_for_path > max_mean_reward_per_step_for_path:
                max_tap_cords = None # meaning we don't need to go through a tap
                max_mean_reward_per_step_for_path = mean_reward_per_step_for_path

        return (max_mean_reward_per_step_for_path, max_tap_cords)

    # Iterate over every robot & plant pair. Find the pair with the highest per step reward,
    # then perform the first step in that plan. Then repeat the first step (finding the optimal path).
    # this plan will likely remain the one with the highest reward per step rate
    # (the reward per step probably increased if we took the right step), so we will probably follow it even if we allow
    # shifting to another plan.
    def find_greedy_best_robot_plant(self, robots: list[Robot], plants: list[Plant], taps: list[Tap]):
        max_mean_reward_per_step_for_path = -float('inf')
        max_tap_cords = None
        max_robot = None
        max_plant = None

        for robot in robots:
            (robot_id, robot_cords, load) = robot
            other_robots = list(_robot for _robot in robots if _robot[0] != robot_id)
            for plant in plants:
                (mean_reward_per_step_for_path, tap_cords) = self.eval_robot_plant(robot, plant, taps, other_robots)
                if max_mean_reward_per_step_for_path < mean_reward_per_step_for_path:
                    max_mean_reward_per_step_for_path = mean_reward_per_step_for_path
                    max_tap_cords = tap_cords
                    max_robot = robot
                    max_plant = plant
        if max_mean_reward_per_step_for_path <= 0 or max_plant is None or max_robot is None:
            # get random action
            return None
        return (max_robot, max_plant, max_tap_cords)

    def get_movement_action_in_path(self, src: Cords, dest: Cords):
        bfs_path = self.bfs_paths.get((dest, src), None)
        if bfs_path is None:
            raise Exception(f"Path from {src} to {dest} does not exist in self.bfs_paths")

        directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
        (r, c) = src
        for (dr, dc), action_name in directions:
            potential_next_step = (r + dr, c + dc)
            if potential_next_step in bfs_path:
                return action_name
