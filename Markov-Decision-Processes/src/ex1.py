import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

KEY_SIZE    = "Size"
KEY_WALLS   = "Walls"
KEY_TAPS    = "Taps"
KEY_PLANTS  = "Plants"
KEY_ROBOTS  = "Robots"

class State:
    __slots__ = (
        'taps', 'plants', 'robots', 'robot_cords', 'robot_cords_tuple',
        'active_robot', 'robot_last_actions', 'non_satiated_plants_cords',
        'non_empty_tap_cords', 'total_plant_water_needed',
        'total_water_available', 'total_load',
        '__hash', '__hash_taps', '__hash_plants',
        '__hash_robots', '__hash_robot_cords_tuple'
    )

    taps                        : tuple[int]
    plants                      : tuple[int]
    robots                      : tuple[tuple[str, int, int]]
    robot_cords                 : set[tuple[int, int]]
    robot_cords_tuple           : tuple[tuple[int, int]]

    # active_robot                = None # int | None = None
    robot_last_actions          : tuple[str] 

    non_satiated_plants_cords   : list[tuple[int, int]]
    non_empty_tap_cords         : list[tuple[int, int]]

    total_plant_water_needed    : int
    total_water_available       : int
    total_load                  : int

    __hash                      : int
    __hash_taps                 : int
    __hash_plants               : int
    __hash_robots               : int
    __hash_robot_cords_tuple    : int


    def __init__(self,
                _old_state                  = None,
                _taps                       = None,#: tuple[int]                    | None  = None,
                _plants                     = None,#: tuple[int]                    | None  = None,
                _robot_cords                = None,#: set[tuple[int, int]]          | None  = None,
                _robot_cords_tuple          = None,#: tuple[tuple[int, int]]        | None  = None,
                _robots                     = None,#: tuple[tuple[str, int, int]]   | None  = None,
                _total_plant_water_needed   = None,#: int                           | None  = None,
                _total_water_available      = None,#: int                           | None  = None,
                _total_load                 = None,#: int                           | None  = None,
                _non_satiated_plants_cords  = None,#: list[tuple[int, int]]         | None  = None,
                _non_empty_tap_cords        = None,#: list[tuple[int, int]]         | None  = None,
                _robot_last_actions         = None,#: tuple[str]                    | None  = None,
                _active_robot               = None):#: int                           | None  = None):

        if _old_state is not None:
            self.taps                       = _old_state.taps
            self.plants                     = _old_state.plants
            self.robot_cords                = _old_state.robot_cords
            self.robot_cords_tuple          = _old_state.robot_cords_tuple
            self.robots                     = _old_state.robots

            self.__hash_taps                = _old_state.__hash_taps
            self.__hash_plants              = _old_state.__hash_plants
            self.__hash_robots              = _old_state.__hash_robots
            self.__hash_robot_cords_tuple   = _old_state.__hash_robot_cords_tuple

            self.total_plant_water_needed   = _old_state.total_plant_water_needed
            self.total_water_available      = _old_state.total_water_available
            self.total_load                 = _old_state.total_load

            self.non_satiated_plants_cords  = _old_state.non_satiated_plants_cords
            self.non_empty_tap_cords        = _old_state.non_empty_tap_cords

        if _non_satiated_plants_cords is not None:
            self.non_satiated_plants_cords  = _non_satiated_plants_cords

        if _non_empty_tap_cords is not None:
            self.non_empty_tap_cords        = _non_empty_tap_cords

        if _taps is not None:
            self.taps                       = _taps
            self.__hash_taps                = hash(self.taps)
            self.total_water_available      = _total_water_available if _total_water_available is not None else sum(_taps)

        if _plants is not None:
            self.plants                     = _plants
            self.__hash_plants              = hash(self.plants)
            self.total_plant_water_needed   = _total_plant_water_needed if _total_plant_water_needed is not None else sum(_plants)

        if _robot_cords is not None:
            self.robot_cords                = _robot_cords

        if _robot_cords_tuple is not None:
            self.robot_cords_tuple          = _robot_cords_tuple
            self.__hash_robot_cords_tuple   = hash(self.robot_cords_tuple)

        if _robots is not None:
            self.robots                     = _robots
            self.__hash_robots              = hash(self.robots)
            self.total_load                 = _total_load if _total_load is not None else sum(load for (id, load, capacity) in _robots)

        if _robot_last_actions is not None:
            self.robot_last_actions         = _robot_last_actions

        self.active_robot                   = _active_robot

        self.__hash = hash((self.__hash_taps, self.__hash_plants, self.__hash_robots,  self.__hash_robot_cords_tuple))

    def __hash__(self):
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
    heuristics_cache:   dict[State, tuple[float, int]] # float is the heuristic itself, int is the least number of steps taken in order to reach this state
    map:                dict[tuple[int, int], tuple[str, int]]
    plant_cords_list:   list[tuple[int, int]]
    tap_cords_list:     list[tuple[int, int]]
    bfs_distances:      dict[tuple[ tuple[int, int], tuple[int, int] ], int]
    plant_tap_distances: dict[tuple[ tuple[int, int], tuple[int, int] ], int]
    bfs_to_whatever_plant_distances:      dict[tuple[int, int], int]

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

        taps                        = tuple(tap_values)
        plants                      = tuple(plant_values)
        non_empty_taps              = [tap_cords    for tap_cords       in self.tap_cords_list      if taps[self.map[tap_cords][1]] > 0]
        non_satiated_plants_cords   = [plant_cords  for plant_cords     in self.plant_cords_list    if plants[self.map[plant_cords][1]] > 0]
        robot_last_actions          = ("",) * len(robots)
        self.initial                = State(_taps               = tuple(tap_values),
                                            _plants             = tuple(plant_values),
                                            _robot_cords        = robot_cords,
                                            _robot_cords_tuple  = robot_cords_tuple,
                                            _robots             = robots,
                                            _non_empty_tap_cords     = non_empty_taps,
                                            _non_satiated_plants_cords = non_satiated_plants_cords,
                                            _robot_last_actions = robot_last_actions)
        self.bfs_distances                      = {}
        self.bfs_to_whatever_plant_distances    = {}
        for cords in self.tap_cords_list:
            self.bfs([cords])
        for cords in self.plant_cords_list:
            self.bfs([cords])
        self.bfs_to_any(self.plant_cords_list)

    def bfs_to_any(self, src_cords: list[tuple[int, int]]): # this would get us the distances between certain coordinates and WHATEVER plant
        """Expects the coordinates of a plant / tap, and in return calculates the minimal distance from the entity to any point lying on the map"""
        visited_nodes: set[tuple[int, int]] = set(src_cords) 
        queue: utils.FIFOQueue              = utils.FIFOQueue()

        queue.extend((cords, 0) for cords in src_cords)

        (height, width) = self.size
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and self.map.get((i,j), (None, -1))[0] != "wall")

        for cords in src_cords:
            self.bfs_to_whatever_plant_distances[cords] = 0
        possible_actions = [(0,-1), (0,1), (-1, 0), (1,0)]

        while (len(queue) > 0):
            (node_i, node_j), cost = queue.pop()
            for (d_i, d_j) in possible_actions:
                if (is_position_legal(node_i + d_i, node_j + d_j)
                    and not (node_i + d_i, node_j + d_j) in visited_nodes):

                    queue.append(((node_i + d_i, node_j + d_j), cost + 1))
                    self.bfs_to_whatever_plant_distances[(node_i + d_i, node_j + d_j)] = cost + 1
                    visited_nodes.add((node_i + d_i, node_j + d_j))

    def bfs(self, src_cords: list[tuple[int, int]]): # accepts a list, as we may have several src points (all plants to whatever taps, but single tap to whatever point)
        """Expects the coordinates of a plant / tap, and in return calculates the minimal distance from the entity to any point lying on the map"""
        visited_nodes: set[tuple[int, int]] = set(src_cords) 
        queue: utils.FIFOQueue              = utils.FIFOQueue()

        queue.extend((cords, 0) for cords in src_cords)

        (height, width) = self.size
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and self.map.get((i,j), (None, -1))[0] != "wall")

        for cords in src_cords:
            self.bfs_distances[(cords, cords)] = 0
        possible_actions = [(0,-1), (0,1), (-1, 0), (1,0)]

        while (len(queue) > 0):
            (node_i, node_j), cost = queue.pop()
            for (d_i, d_j) in possible_actions:
                if (is_position_legal(node_i + d_i, node_j + d_j)
                    and not (node_i + d_i, node_j + d_j) in visited_nodes):

                    queue.append(((node_i + d_i, node_j + d_j), cost + 1))
                    for cords in src_cords:
                        self.bfs_distances[(cords, (node_i + d_i, node_j + d_j))] = cost + 1
                    visited_nodes.add((node_i + d_i, node_j + d_j))

    # def distance_nearest_plant(self, cords):
    #     dist = self.bfs_to_whatever_plant_distances.get(cords, None)
    #     if dist is None: # this either means that the destination was unreachable, or that the function was not used properly
    #         return float('inf')
    #     return dist

    # def distance(self, cords_1: tuple[int, int], cords_2: tuple[int, int]):
    #     dist = self.bfs_distances.get((cords_1, cords_2), None)
    #     if dist is None: # this either means that the destination was unreachable, or that the function was not used properly
    #         return float('inf')
    #     return dist 

    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        (height, width) = self.size
        moves = []
        # is_move_legal = lambda i,j: ( I INLINED EVERYTHING FOR THE SUBMISSION, 
        #         (0 <= i < height)
        #         and (0 <= j < width)
        #         and self.map.get((i,j), (None, -1))[0] != "wall")

        #is_there_robot          = lambda i,j: (i,j) in state.robot_cords

        # tuple_replace           = lambda t, index, new_value: t[:index] + (new_value,) + t[index+1:]
        # tuple_remove            = lambda t, index: t[:index] + t[index+1:]

        map                     = self.map
        plant_cords_list        = self.plant_cords_list
        tap_cords_list          = self.tap_cords_list


        for index, (id, load, capacity) in enumerate(state.robots):
            (i,j) = state.robot_cords_tuple[index]
            entity_res          = None

            if load > 0:
                (entity_type, entity_index) = entity_res = map.get((i,j), (None, -1))
                if entity_type == "plant":
                    plant_water_needed = state.plants[entity_index]
                    if plant_water_needed > 0:
                        state_new_robots        = state.robots[:index] + ((id, load - 1, capacity),) + state.robots[index+1:]
                        state_new_plants        = state.plants[:entity_index] + (plant_water_needed - 1,) + state.plants[entity_index+1:]
                        action_name             = f"POUR{{{id}}}"
                        state_new_last_actions  = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                        non_satiated_plants_cords       = None
                        if plant_water_needed == 1:
                            non_satiated_plants_cords   = [plant_cords  for plant_cords     in plant_cords_list    if state_new_plants[map[plant_cords][1]] > 0]

                        new_active_robot        = None if load - 1 == 0 else index
                        state_new               = State(state,
                                                    _plants = state_new_plants,
                                                    _robots = state_new_robots,
                                                    _total_load = state.total_load - 1,
                                                    _total_plant_water_needed = state.total_plant_water_needed - 1,
                                                    _non_satiated_plants_cords = non_satiated_plants_cords,
                                                    _robot_last_actions = state_new_last_actions,
                                                    _active_robot       = new_active_robot)
                        if self.heuristics_cache.get(state_new, None) is None:
                            moves.append((action_name, state_new))
                        if len(state.robots) == 1 or len(state.non_satiated_plants_cords) == 1 or load >= state.total_plant_water_needed:
                            continue

            if state.active_robot is None or state.active_robot == index:
                remaining_capacity = capacity - load
                if remaining_capacity > 0 and state.total_load < state.total_plant_water_needed:
                    (entity_type, entity_index) = entity_res if entity_res else map.get((i,j), (None, -1))
                    if entity_type == "tap":
                        water_available = state.taps[entity_index]
                        if water_available > 0:
                            state_new_robots    = state.robots[:index] + ((id, load + 1, capacity),) + state.robots[index+1:]
                            state_new_taps      = state.taps[:entity_index] + (water_available - 1,) + state.taps[entity_index+1:]
                            action_name = f"LOAD{{{id}}}"
                            state_new_last_actions = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                            state_new_non_empty_taps = None
                            if water_available == 1:
                                state_new_non_empty_taps = [tap_cords    for tap_cords       in tap_cords_list      if state_new_taps[map[tap_cords][1]] > 0]

                            state_new           = State(state,
                                                    _robots                 = state_new_robots,
                                                    _taps                   = state_new_taps,
                                                    _total_load             = state.total_load + 1,
                                                    _total_water_available  = state.total_water_available - 1,
                                                    _non_empty_tap_cords    = state_new_non_empty_taps,
                                                    _robot_last_actions     = state_new_last_actions,
                                                    _active_robot           = index)

                            if self.heuristics_cache.get(state_new, None) is None:
                                moves.append((action_name, state_new))
                            if len(state.robots) == 1:
                                continue
                            if (len(state.non_empty_tap_cords) == 1 and state.robot_last_actions[index] == action_name) and load < min(state.plants): # and last action was LOAD, then keep LOADing
                                continue

            if (
                (0 <= i-1 < height)
                and (0 <= j < width)
                and self.map.get((i-1,j), (None, -1))[0] != "wall"):
                opposite_action = f"DOWN{{{id}}}"
                if (i+1,j) in state.robot_cords:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = state.robot_last_actions[:index] + ("",) + state.robot_last_actions[index+1:]
                if not (i-1,j) in state.robot_cords and state.robot_last_actions[index] != opposite_action:
                    state_new_robot_cords_tuple = state.robot_cords_tuple[:index] + ((i-1,j),) + state.robot_cords_tuple[index+1:]
                    state_new_robot_cords   = set(state_new_robot_cords_tuple)
                    action_name             = f"UP{{{id}}}"
                    state_new_last_actions = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                    state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot)

                    if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if (
                (0 <= i+1 < height)
                and (0 <= j < width)
                and self.map.get((i+1,j), (None, -1))[0] != "wall"):
                opposite_action = f"UP{{{id}}}"
                if (i-1,j) in state.robot_cords:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = state.robot_last_actions[:index] + ("",) + state.robot_last_actions[index+1:]
                if not (i+1,j) in state.robot_cords and state.robot_last_actions[index] != opposite_action:
                    state_new_robot_cords_tuple = state.robot_cords_tuple[:index] + ((i+1,j),) + state.robot_cords_tuple[index+1:]
                    state_new_robot_cords   = set(state_new_robot_cords_tuple)
                    action_name             = f"DOWN{{{id}}}"
                    state_new_last_actions = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                    state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot)

                    if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if (
                (0 <= i < height)
                and (0 <= j-1 < width)
                and self.map.get((i,j-1), (None, -1))[0] != "wall"):
                opposite_action = f"RIGHT{{{id}}}"
                if (i,j+1) in state.robot_cords:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = state.robot_last_actions[:index] + ("",) + state.robot_last_actions[index+1:]
                if not (i,j-1) in state.robot_cords and state.robot_last_actions[index] != opposite_action:
                    state_new_robot_cords_tuple = state.robot_cords_tuple[:index] + ((i,j-1),) + state.robot_cords_tuple[index+1:]
                    state_new_robot_cords   = set(state_new_robot_cords_tuple)
                    action_name             = f"LEFT{{{id}}}"
                    state_new_last_actions = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                    state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot)

                    if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if (
                (0 <= i < height)
                and (0 <= j+1 < width)
                and self.map.get((i,j+1), (None, -1))[0] != "wall"):
                opposite_action = f"LEFT{{{id}}}"
                if (i,j-1) in state.robot_cords:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = state.robot_last_actions[:index] + ("",) + state.robot_last_actions[index+1:]
                if not (i,j+1) in state.robot_cords and state.robot_last_actions[index] != opposite_action:
                    state_new_robot_cords_tuple = state.robot_cords_tuple[:index] + ((i,j+1),) + state.robot_cords_tuple[index+1:]
                    state_new_robot_cords   = set(state_new_robot_cords_tuple)
                    action_name             = f"RIGHT{{{id}}}"
                    state_new_last_actions = state.robot_last_actions[:index] + (action_name,) + state.robot_last_actions[index+1:]
                    state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot)

                    if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

        return moves

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return all(not plant_water_needed for plant_water_needed in state.plants)
    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        INF                             = float('inf')
        state                           = node.state
        total_load                      = state.total_load
        total_plant_water_needed        = state.total_plant_water_needed
        total_water_available           = state.total_water_available

        # Cheaper than querying the cache...
        if not state.non_satiated_plants_cords:
            return 0
        if total_water_available + total_load < total_plant_water_needed:
            return INF

        cached_result = self.heuristics_cache.get(state, None)
        if cached_result is not None:
            (value, path_cost)          = cached_result
            if path_cost < node.path_cost:
                return value
                #return INF
            else:
                self.heuristics_cache[state] = (value, node.path_cost)
                return value
 
        min_robot_contribution_distance         = INF
        get_dist                                = self.bfs_distances.get
        get_plant_dist                          = self.bfs_to_whatever_plant_distances.get

        for index, (id, load, capacity) in enumerate(state.robots):
            robot_cords = state.robot_cords_tuple[index]
            current_robot_contribution_distance = INF
            distance_tap_then_plant = INF

            if state.non_empty_tap_cords:
                distance_tap_then_plant = min(
                        (get_dist((tap_cords, robot_cords), INF)) + get_plant_dist(tap_cords, INF)
                        for tap_cords in state.non_empty_tap_cords)

            if load == 0:
                    current_robot_contribution_distance = distance_tap_then_plant
            else:
                if total_load < total_plant_water_needed:
                    current_robot_contribution_distance = INF
                    if state.non_empty_tap_cords and state.non_satiated_plants_cords:
                        current_robot_contribution_distance  = min(
                                (get_dist((plant_cords, robot_cords), INF)) + (get_dist((plant_cords, tap_cords), INF)) # for some reason calculating the distance for the second plant makes it slower
                                for tap_cords       in state.non_empty_tap_cords
                                for plant_cords     in state.non_satiated_plants_cords)
                    current_robot_contribution_distance = min(distance_tap_then_plant, current_robot_contribution_distance)
                else:
                    current_robot_contribution_distance  = get_plant_dist(robot_cords, INF)
            min_robot_contribution_distance          = min(min_robot_contribution_distance, current_robot_contribution_distance)

        heuristic                       = 2 * total_plant_water_needed - total_load

        # maybe add a clause saying that if we do not have enough water, e.g: total_load < total_plant_water_needed then we need to add the amount of water to get to a plant

        heuristic += min_robot_contribution_distance

        self.heuristics_cache[state] = (heuristic, node.path_cost)
        return heuristic

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        return 2 * node.state.total_plant_water_needed - sum(load for id, load, capacity in node.state.robots)

def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
