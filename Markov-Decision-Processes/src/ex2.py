import ext_plant
import numpy as np
from collections import deque

id = ["000000000"]

Cords = tuple[int, int]
Robot = tuple[int, Cords, int]
Plant = tuple[Cords, int]

class Controller:
    """This class is a controller for the ext_plant game."""

    original_game: ext_plant.Game
    bfs_distances: dict[tuple[Cords, Cords], int]
    bfs_paths:          dict[tuple[ tuple[int, int], tuple[int, int] ], set[tuple[int, int]]]

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game
        self.bfs_path = {}
        self.bfs_distances = {}
  
    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        possible_moves = []

        (robots, plants, taps, total_water_need) = state
        capacities = self.original_game.get_capacities()

        directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
        for robot_id, (r, c), load in robots:
            remaining_capacity = capacities[robot_id] - load
            for (dr, dc), action_name in directions:
                destination = (r + dr, c + dc)
                if (all(cords != destination for (_, cords, __) in robots) and
                    destination not in self.original_game.walls and
                    0 <= r + dr < self.original_game.rows and
                    0 <= c + dc < self.original_game.cols):
                    possible_moves.append(f"{action_name}({robot_id})")
            if load > 0:
                plant_in_current_location = next(((pos, need) for (pos, need) in plants if (r, c) == pos and need > 0), None)
                if plant_in_current_location is not None:
                    possible_moves.append(f"POUR({robot_id})")
            if remaining_capacity > 0:
                tap_in_current_location = next(((pos, available_water) for (pos, available_water) in taps if (r, c) == pos and available_water > 0), None)
                if tap_in_current_location is not None:
                    possible_moves.append(f"LOAD({robot_id})")
        return np.random.choice(np.array(possible_moves))

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
# M := min(tap_available_water, plant_water_needed / success_rate, robot_capacity)
# * plant_water_needed / success_rate is the EXPECTED amount of water needed to satiate the plant with this particular robot
# now, get the amount of steps needed to get to the tap, and from the tap to the plant using calc_mean_step that we worked on.
# get the mean of the total number of turns as being: (M / success_rate) loading operations (when failure occurs, we don't load anything but just waste a turn) + the mean amount of steps required to get to the tap and from there to the plant + M (the amount of times we POUR on the plant)
# now, get the mean reward for this option as: M * success_rate * plant_reward_per_water_unit
# at last, divide the mean reward for this option by the mean total amount of turns  to get the mean per step reward
# from this we find the best greedy option for going to a tap then to a plant, we will later compare it with going directly to the plant.
    def eval_robot_plant(self, robot: Robot, plant: Plant):
        success_rate
