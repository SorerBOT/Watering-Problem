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
    taps:                   tuple[int]
    plants:                 tuple[int]
    robots:                 tuple[tuple[str, int, int]]
    robot_cords:            set[tuple[int, int]]
    robot_cords_tuple:      tuple[tuple[int, int]]
    __hash:                 int | None

    def __init__(self,
                _old_state                                                  = None,
                _taps               : tuple[int]                    | None  = None,
                _plants             : tuple[int]                    | None  = None,
                _robot_cords        : set[tuple[int, int]]    | None  = None,
                _robot_cords_tuple  : tuple[tuple[int, int]]        | None  = None,
                _robots             : tuple[tuple[str, int, int]]   | None  = None):

        if _old_state is not None:
            self.taps               = _old_state.taps
            self.plants             = _old_state.plants
            self.robot_cords        = _old_state.robot_cords
            self.robot_cords_tuple  = _old_state.robot_cords_tuple
            self.robots             = _old_state.robots

        if _taps is not None:
            self.taps           = _taps
        if _plants is not None:
            self.plants         = _plants
        if _robot_cords is not None:
            self.robot_cords    = _robot_cords
        if _robot_cords_tuple is not None:
            self.robot_cords_tuple    = _robot_cords_tuple
        if _robots is not None:
            self.robots         = _robots

        self.__hash             = None

    def __hash__(self):
        if self.__hash is not None:
            return self.__hash
        else:
            self.__hash = hash((
                self.taps,
                self.plants,
                self.robots,
                self.robot_cords_tuple))
        return self.__hash

    def __eq__(self, other):
        return (
            self.taps   == other.taps   and
            self.plants == other.plants and
            self.robots == other.robots and
            self.robot_cords_tuple == other.robot_cords_tuple)

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    size:               tuple[int, int]
    initial:            State
    heuristics_cache:   dict[State, float]
    map:                dict[tuple[int, int], tuple[str, int]]
    plant_cords_list:   list[tuple[int, int]]
    tap_cords_list:     list[tuple[int, int]]

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.heuristics_cache           = {}
        self.size                       = initial[KEY_SIZE]
        self.map                        = {}
        plant_values                    = []
        tap_values                      = []
        self.tap_cords_list             = []
        self.plant_cords_list           = []

        robot_cords_tuple               = tuple((i,j) for id, (i, j, load, capacity) in initial[KEY_ROBOTS].items())
        robot_cords                     = set(robot_cords_tuple)
        robots                          = tuple((str(id), load, capacity) for id, (i, j, load, capacity) in initial[KEY_ROBOTS].items())

        for index, (plant_cords, plant_water_needed) in enumerate(initial[KEY_PLANTS].items()):
            self.map[plant_cords]   = ("plant", index)
            plant_values.append(plant_water_needed)
            self.plant_cords_list.append(plant_cords)
        for index, (tap_cords, water_available) in enumerate(initial[KEY_TAPS].items()):
            self.map[tap_cords]     = ("tap", index)
            tap_values.append(water_available)
            self.tap_cords_list.append(tap_cords)
        for wall_cords in initial[KEY_WALLS]:
            self.map[wall_cords]    = ("wall", -1)

        self.initial                = State(_taps               = tuple(tap_values),
                                            _plants             = tuple(plant_values),
                                            _robot_cords        = robot_cords,
                                            _robot_cords_tuple  = robot_cords_tuple,
                                            _robots             = robots)

    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        (height, width) = self.size
        moves = []
        is_move_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and self.map.get((i,j), (None, -1))[0] != "wall"
                and (i,j) not in state.robot_cords)

        tuple_replace           = lambda t, index, new_value: t[:index] + (new_value,) + t[index+1:]

        for index, (id, load, capacity) in enumerate(state.robots):
            (i,j) = state.robot_cords_tuple[index]

            if load > 0:
                (entity_type, entity_index) = self.map.get((i,j), (None, -1))
                if entity_type == "plant":
                    plant_water_needed = state.plants[entity_index]
                    if plant_water_needed > 0:
                        state_new_robots        = tuple_replace(state.robots, index, (id, load - 1, capacity))
                        state_new_plants        = tuple_replace(state.plants, entity_index, plant_water_needed - 1)
                        state_new               = State(state,
                                                    _plants = state_new_plants,
                                                    _robots = state_new_robots)
                        if self.heuristics_cache.get(state_new, None) is None:
                            moves.append((f"POUR{{{id}}}", state_new))
                            if len(state.robots) == 1:
                                continue

            remaining_capacity = capacity - load
            if remaining_capacity > 0:
                (entity_type, entity_index) = self.map.get((i,j), (None, -1))
                if entity_type == "tap":
                    water_available = state.taps[entity_index]
                    if water_available > 0:
                        state_new_robots    = tuple_replace(state.robots, index, (id, load + 1, capacity))
                        state_new_taps      = tuple_replace(state.taps, entity_index, water_available - 1)
                        state_new           = State(state,
                                                _robots = state_new_robots,
                                                _taps   = state_new_taps)

                        if self.heuristics_cache.get(state_new, None) is None:
                            moves.append((f"LOAD{{{id}}}", state_new))
                            if len(state.robots) == 1:
                                continue
            if is_move_legal(i+1, j):
                state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i+1,j))
                state_new_robot_cords   = set(state_new_robot_cords_tuple)
                state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple)

                if self.heuristics_cache.get(state_new, None) is None:
                    moves.append((f"DOWN{{{id}}}", state_new))

            if is_move_legal(i-1, j):
                state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i-1,j))
                state_new_robot_cords   = set(state_new_robot_cords_tuple)
                state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple)

                if self.heuristics_cache.get(state_new, None) is None:
                    moves.append((f"UP{{{id}}}", state_new))

            if is_move_legal(i, j+1):
                state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i,j+1))
                state_new_robot_cords   = set(state_new_robot_cords_tuple)
                state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple)

                if self.heuristics_cache.get(state_new, None) is None:
                    moves.append((f"RIGHT{{{id}}}", state_new))

            if is_move_legal(i, j-1):
                state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i,j-1))
                state_new_robot_cords   = set(state_new_robot_cords_tuple)
                state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple)

                if self.heuristics_cache.get(state_new, None) is None:
                    moves.append((f"LEFT{{{id}}}", state_new))

        return moves

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return all(not plant_water_needed for plant_water_needed in state.plants)

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        # cached_result = self.heuristics_cache.get(node.state, None)
        # if cached_result is not None:
        #         return cached_result

        total_load                              = 0
        non_satiated_plants_cords               = [plant_cords  for plant_cords     in self.plant_cords_list    if node.state.plants[self.map[plant_cords][1]] > 0]
        non_empty_tap_cords                     = [tap_cords    for tap_cords       in self.tap_cords_list      if node.state.taps[self.map[tap_cords][1]] > 0]

        if not non_satiated_plants_cords:
            return 0

        min_robot_contribution_distance         = float('inf')
        for index, (id, load, capacity) in enumerate(node.state.robots):
            robot_cords = node.state.robot_cords_tuple[index]
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
            total_load                              += load

        total_water_available           = sum(node.state.taps)
        total_plant_water_needed        = sum(node.state.plants)
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
        return 2 * sum(node.state.plants) - sum(load for id, load, capacity in node.state.robots)

def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
