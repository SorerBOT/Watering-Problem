import ext_plant
import numpy as np
from collections import deque

id = ["000000000"]

Cords = tuple[int, int]

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
        for cords in game.taps:
            self.bfs(cords)
        for cords in game.plants:
            self.bfs(cords)
  
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

    def bfs(self, cords: tuple[int, int]): # accepts a list, as we may have several src points (all plants to whatever taps, but single tap to whatever point)
        """Expects the coordinates of a plant / tap, and in return calculates the minimal distance from the entity to any point lying on the map"""
        visited_nodes: set[tuple[int, int]] = set({ cords }) 
        queue = deque()

        queue.append(cords)

        (height, width) = self.original_game.rows, self.original_game.cols
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and (i,j) not in self.original_game.walls)

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
