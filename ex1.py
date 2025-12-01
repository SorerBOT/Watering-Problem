import math
import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

KEY_SIZE    = "Size"
KEY_WALLS   = "Walls"
KEY_TAPS    = "Taps"
KEY_PLANTS  = "Plants"
KEY_ROBOTS  = "Robots"

def distance(cords_1: tuple[int, int], cords_2: tuple[int, int]):
    return abs(cords_1[0] - cords_2[0]) + abs(cords_1[1] - cords_2[1])

class State:
    size:           tuple[int, int]
    walls:          dict[tuple[int, int], bool]
    taps:           dict[tuple[int, int], int]
    plants:         dict[tuple[int, int], int]
    robots:         dict[tuple[int, int], tuple[str, int, int]]
    __hash:         int | None

    @staticmethod
    def create_initial_state(initial):
        State.size      = tuple(initial[KEY_SIZE])
        State.walls     = dict([((i,j), True) for (i,j) in initial[KEY_WALLS]]) 
        _robots         = dict(((i,j), (str(id), load, capacity)) for id, (i, j, load, capacity) in initial[KEY_ROBOTS].items())

        return State(None,
                     initial[KEY_TAPS],
                     initial[KEY_PLANTS],
                     _robots)
    def __init__(self,
                _old_state                                  = None,
                _taps       : dict[tuple, int]      | None  = None,
                _plants     : dict[tuple, int]      | None  = None,
                _robots     : dict[tuple, tuple]    | None  = None):

        if _old_state is not None:
            self.taps       = _old_state.taps
            self.plants     = _old_state.plants
            self.robots     = _old_state.robots

        if _taps is not None:
            self.taps           = dict(_taps)
        if _plants is not None:
            self.plants         = dict(_plants)
        if _robots is not None:
            self.robots         = dict(_robots)
        self.__hash         = None

    def __str__(self):
        str_size                = f"Grid dimensions: {State.size}"
        str_walls               = f"Walls coordinates: {State.walls}"
        str_taps                = f"Taps: {self.taps}"
        str_plants              = f"Plants: {self.plants}"
        str_robots              = f"Robots: {self.robots}"

        return f"{str_size}\n{str_walls}\n{str_taps}\n{str_plants}\n{str_robots}"

    def __hash__(self):
        if self.__hash:
            return self.__hash
        else:
            self.__hash = hash((
                tuple(sorted(self.taps.items())),
                tuple(sorted(self.plants.items())),
                tuple(sorted(self.robots.items())),
                ))
        return self.__hash

    def __eq__(self, other):
        return (
            self.taps   == other.taps   and
            self.plants == other.plants and
            self.robots == other.robots)

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    initial:            State
    heuristics_cache:   dict[State, float]
    cache_hits:         int
    cache_misses:       int

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.initial                    = State.create_initial_state(initial)
        self.heuristics_cache           = {}
        self.cache_hits                 = 0
        self.cache_misses               = 0

    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        (height, width) = state.size
        moves = []
        is_move_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and not State.walls.get((i,j), False)
                and not state.robots.get((i,j), False))

        # -------------------------------------------------------------------
        # Didn't want to commit to a specific format for saving the robots. #
        # -------------------------------------------------------------------
        generate_robot          = lambda id, i, j, load, capacity: (id, load, capacity)
        generate_robot_key      = lambda id, i, j, load, capacity: (i,j)

        for (i,j), (id, load, capacity) in state.robots.items():
            old_robot_key           = generate_robot_key(id, i, j, load, capacity)

            if (load > 0):
                plant_water_needed = state.plants.get((i,j), 0)
                if plant_water_needed > 0:
                    state_new_robots    = dict(state.robots)
                    state_new_plants    = dict(state.plants)

                    del state_new_robots[old_robot_key]
                    state_new_robots[generate_robot_key(id, i, j, load - 1, capacity)] = generate_robot(id, i, j, load - 1, capacity)
                    state_new_plants[(i,j)] = plant_water_needed - 1
                    state_new               = State(state,
                                                _plants = state_new_plants,
                                                _robots = state_new_robots)

                    moves.append((f"POUR{{{id}}}", state_new))
                    if len(state.robots.items()) == 1:
                        continue

            remaining_capacity = capacity - load
            if (remaining_capacity > 0):
                water_available = state.taps.get((i,j), 0)
                if water_available > 0:
                    state_new_robots    = dict(state.robots)
                    state_new_taps      = dict(state.taps)

                    del state_new_robots[old_robot_key]
                    state_new_robots[generate_robot_key(id, i, j, load + 1, capacity)] = generate_robot(id, i, j, load + 1, capacity)
                    state_new_taps[(i,j)] = water_available - 1

                    state_new = State(state,
                                    _robots = state_new_robots,
                                    _taps   = state_new_taps)

                    moves.append((f"LOAD{{{id}}}", state_new))
                    if len(state.robots.items()) == 1 and load <= sum(state.plants.values()):
                        continue
            if is_move_legal(i+1, j):
                state_new_robots = dict(state.robots)
                del state_new_robots[old_robot_key]
                state_new_robots[generate_robot_key(id, i+1, j, load, capacity)] = generate_robot(id, i+1, j, load, capacity)
                state_new       = State(state, _robots = state_new_robots)

                moves.append((f"DOWN{{{id}}}", state_new))

            if is_move_legal(i-1, j):
                state_new_robots    = dict(state.robots)
                del state_new_robots[old_robot_key]
                state_new_robots[generate_robot_key(id, i-1, j, load, capacity)] = generate_robot(id, i-1, j, load, capacity)
                state_new           = State(state, _robots = state_new_robots)

                moves.append((f"UP{{{id}}}", state_new))

            if is_move_legal(i, j+1):
                state_new_robots    = dict(state.robots)
                del state_new_robots[old_robot_key]
                state_new_robots[generate_robot_key(id, i, j+1, load, capacity)] = generate_robot(id, i, j+1, load, capacity)
                state_new           = State(state, _robots = state_new_robots)

                moves.append((f"RIGHT{{{id}}}", state_new))

            if is_move_legal(i, j-1):
                state_new_robots    = dict(state.robots)
                del state_new_robots[old_robot_key]
                state_new_robots[generate_robot_key(id, i, j-1, load, capacity)] = generate_robot(id, i, j-1, load, capacity)
                state_new           = State(state, _robots = state_new_robots)

                moves.append((f"LEFT{{{id}}}", state_new))

        return moves

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return all(not v for v in state.plants.values())

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        cached_result = self.heuristics_cache.get(node.state, None)
        if cached_result is not None:
                self.cache_hits         += 1
                return cached_result
        else:
                self.cache_misses       += 1

        total_load                              = 0
        non_satiated_plants_cords               = [plant_cords  for plant_cords, plant_water_needed in node.state.plants.items()    if plant_water_needed > 0]
        non_empty_tap_cords                     = [tap_cords    for tap_cords, water_available      in node.state.taps.items()      if water_available > 0]

        if not non_satiated_plants_cords:
            return 0

        min_robot_contribution_distance         = float('inf')
        for robot_cords, (id, load, capacity) in node.state.robots.items():
            current_robot_contribution_distance = float('inf')
            if load == 0:
                if non_empty_tap_cords:
                    current_robot_contribution_distance  = min(
                        distance(robot_cords, tap_cords) + distance(tap_cords, plant_cords)
                        for tap_cords in non_empty_tap_cords
                        for plant_cords in non_satiated_plants_cords)
            else:
                current_robot_contribution_distance  = min(
                        distance(robot_cords, plant_cords)
                        for plant_cords in non_satiated_plants_cords)
            min_robot_contribution_distance          = min(min_robot_contribution_distance, current_robot_contribution_distance)
            total_load                      += load

        total_water_available           = sum(node.state.plants.values())
        total_plant_water_needed        = sum(node.state.plants.values())
        heuristic                       = 2 * total_plant_water_needed - total_load

        if total_water_available + total_load < total_plant_water_needed:
            return float('inf')
        else:
            heuristic += min_robot_contribution_distance

        self.heuristics_cache[node.state] = heuristic
        return heuristic


    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        return 2 * sum(node.state.plants.values()) - sum(load for (id, load, capacity) in node.state.robots.values())

def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
