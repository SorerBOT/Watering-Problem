import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

KEY_SIZE    = "Size"
KEY_WALLS   = "Walls"
KEY_TAPS    = "Taps"
KEY_PLANTS  = "Plants"
KEY_ROBOTS  = "Robots"

# If there is only one ROBOT, we should not calculate BFS distances ? Maybe just use the pruning??

# BFS: calculate distance to all taps as we do at this moment, and then calculate the distance from every tap, to WHATEVER plant using multi-search

#
# 1. Why did pruning visited states work? What about g?
# 2. Heuristic which takes into consideration entire paths (to satiate all plants) instead of the minimum effort in order to contribute.
# 3. Add walls to remove redundant paths
# 4. Can we break admissibility when we already visited the state with a higher g value?
#

class State:
    taps                        : tuple[int]
    plants                      : tuple[int]
    robots                      : tuple[tuple[str, int, int]]
    robot_cords                 : set[tuple[int, int]]
    robot_cords_tuple           : tuple[tuple[int, int]]

    active_robot                : int | None = None
    is_active_only              : bool = False
    destination                 : tuple[int, int] | None = None
    robot_last_actions          : tuple[str]

    non_satiated_plants_cords   : list[tuple[int, int]]
    non_empty_tap_cords         : list[tuple[int, int]]

    total_plant_water_needed    : int
    total_water_available       : int
    total_load                  : int
    #min_plant                   : int # we will use this instead of calculating the minimum need of a plant every time

    __hash                      : int
    __hash_taps                 : int
    __hash_plants               : int
    __hash_robots               : int
    __hash_robot_cords_tuple    : int


    def __init__(self,
                _old_state                                                          = None,
                _taps                       : tuple[int]                    | None  = None,
                _plants                     : tuple[int]                    | None  = None,
                _robot_cords                : set[tuple[int, int]]          | None  = None,
                _robot_cords_tuple          : tuple[tuple[int, int]]        | None  = None,
                _robots                     : tuple[tuple[str, int, int]]   | None  = None,
                _total_plant_water_needed   : int                           | None  = None,
                _total_water_available      : int                           | None  = None,
                _total_load                 : int                           | None  = None,
                _non_satiated_plants_cords  : list[tuple[int, int]]         | None  = None,
                _non_empty_tap_cords        : list[tuple[int, int]]         | None  = None,
                _robot_last_actions         : tuple[str]                    | None  = None,
                _active_robot               : int                           | None  = None,
                _is_active_only             : bool                                  = False,
                _destination                : tuple[int, int]               | None  = None):

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
            self.non_satiated_plants_cords = _non_satiated_plants_cords

        if _non_empty_tap_cords is not None:
            self.non_empty_tap_cords = _non_empty_tap_cords

        if _taps is not None:
            self.taps           = _taps
            self.__hash_taps    = hash(self.taps)
            self.total_water_available   = _total_water_available if _total_water_available is not None else sum(_taps)

        if _plants is not None:
            self.plants         = _plants
            self.__hash_plants    = hash(self.plants)
            self.total_plant_water_needed   = _total_plant_water_needed if _total_plant_water_needed is not None else sum(_plants)

        if _robot_cords is not None:
            self.robot_cords    = _robot_cords

        if _robot_cords_tuple is not None:
            self.robot_cords_tuple    = _robot_cords_tuple
            self.__hash_robot_cords_tuple    = hash(self.robot_cords_tuple)

        if _robots is not None:
            self.robots         = _robots
            self.__hash_robots  = hash(self.robots)
            self.total_load     = _total_load if _total_load is not None else sum(load for (id, load, capacity) in _robots)

        if _robot_last_actions is not None:
            self.robot_last_actions = _robot_last_actions

        self.active_robot   = _active_robot
        self.is_active_only = _is_active_only
        self.destination    = _destination

        self.__hash = hash((self.__hash_taps,
                            self.__hash_plants,
                            self.__hash_robots,
                            self.__hash_robot_cords_tuple,
                            self.destination,
                            self.active_robot,
                            self.is_active_only))

    def __hash__(self):
        return self.__hash

    def __eq__(self, other):
        return (
            self.taps   == other.taps   and
            self.plants == other.plants and
            self.robots == other.robots and
            self.robot_cords_tuple == other.robot_cords_tuple and
            self.destination == other.destination and
            self.active_robot == other.active_robot and
            self.is_active_only == other.is_active_only)

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    size:               tuple[int, int]
    initial:            State
    heuristics_cache:   dict[State, tuple[float, int]] # float is the heuristic itself, int is the least number of steps taken in order to reach this state
    map:                dict[tuple[int, int], tuple[str, int]]
    plant_cords_list:   list[tuple[int, int]]
    tap_cords_list:     list[tuple[int, int]]
    bfs_distances:      dict[tuple[ tuple[int, int], tuple[int, int] ], int]
    bfs_paths:          dict[tuple[ tuple[int, int], tuple[int, int] ], set[tuple[int, int]]]
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
        self.bfs_paths                          = {}
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

        queue.extend(cords for cords in src_cords)

        (height, width) = self.size
        is_position_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and self.map.get((i,j), (None, -1))[0] != "wall")

        for cords in src_cords:
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
                    for cords in src_cords:
                        self.bfs_distances[(cords, new_point)] = self.bfs_distances[(cords, old_point)] + 1
                        self.bfs_paths[(cords, new_point)] = self.bfs_paths[(cords, (node_i, node_j))].union({ new_point })
                    visited_nodes.add((node_i + d_i, node_j + d_j))

    def distance_nearest_plant(self, cords):
        dist = self.bfs_to_whatever_plant_distances.get(cords, None)
        if dist is None: # this either means that the destination was unreachable, or that the function was not used properly
            return float('inf')
        return dist

    def distance(self, cords_1: tuple[int, int], cords_2: tuple[int, int]):
        dist = self.bfs_distances.get((cords_1, cords_2), None)
        if dist is None: # this either means that the destination was unreachable, or that the function was not used properly
            return float('inf')
        return dist 

    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        (height, width) = self.size
        moves = []
        is_move_legal = lambda i,j: (
                (0 <= i < height)
                and (0 <= j < width)
                and self.map.get((i,j), (None, -1))[0] != "wall")

        is_there_robot          = lambda i,j: (i,j) in state.robot_cords

        tuple_replace           = lambda t, index, new_value: t[:index] + (new_value,) + t[index+1:]
        tuple_remove            = lambda t, index: t[:index] + t[index+1:]

        if state.active_robot is not None:
            if state.destination == state.robot_cords_tuple[state.active_robot]:
                state.destination = None
                state.is_active_only = False

        for index, (id, load, capacity) in enumerate(state.robots):
            if state.is_active_only and index != state.active_robot:
                continue

            (i,j) = state.robot_cords_tuple[index]

            current_path = None
            if state.destination:
                current_path = self.bfs_paths[(state.destination, (i,j))]

            entity_res      = None

            if load > 0:
                (entity_type, entity_index) = entity_res = self.map.get((i,j), (None, -1))
                if entity_type == "plant":
                    plant_water_needed = state.plants[entity_index]
                    if plant_water_needed > 0:
                        state_new_robots        = tuple_replace(state.robots, index, (id, load - 1, capacity))
                        state_new_plants        = tuple_replace(state.plants, entity_index, plant_water_needed - 1)
                        action_name             = f"POUR{{{id}}}"
                        state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                        non_satiated_plants_cords       = None
                        if plant_water_needed == 1:
                            non_satiated_plants_cords   = [plant_cords  for plant_cords     in self.plant_cords_list    if state_new_plants[self.map[plant_cords][1]] > 0]

                        new_active_robot                = None if load - 1 == 0 else index
                        is_active_only                  = state.is_active_only and new_active_robot is not None
                        destination                     = state.destination if new_active_robot else None

                        state_new                       = State(state,
                                                            _plants = state_new_plants,
                                                            _robots = state_new_robots,
                                                            _total_load = state.total_load - 1,
                                                            _total_plant_water_needed = state.total_plant_water_needed - 1,
                                                            _non_satiated_plants_cords = non_satiated_plants_cords,
                                                            _robot_last_actions = state_new_last_actions,
                                                            _active_robot       = new_active_robot,
                                                            _is_active_only         = is_active_only,
                                                            _destination            = destination)
                        #if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))
                        if len(state.robots) == 1 or len(state.non_satiated_plants_cords) == 1 or load >= state.total_plant_water_needed:
                            continue

            if state.active_robot is None or state.active_robot == index:
                remaining_capacity = capacity - load
                if remaining_capacity > 0 and state.total_load < state.total_plant_water_needed:
                    (entity_type, entity_index) = entity_res if entity_res else self.map.get((i,j), (None, -1))
                    if entity_type == "tap":
                        water_available = state.taps[entity_index]
                        if water_available > 0:
                            state_new_robots    = tuple_replace(state.robots, index, (id, load + 1, capacity))
                            state_new_taps      = tuple_replace(state.taps, entity_index, water_available - 1)
                            action_name = f"LOAD{{{id}}}"
                            state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                            state_new_non_empty_taps = None
                            if water_available == 1:
                                state_new_non_empty_taps = [tap_cords    for tap_cords       in self.tap_cords_list      if state_new_taps[self.map[tap_cords][1]] > 0]

                            new_states  = []
                            if state.active_robot is not None:
                                state_new       = State(state,
                                                    _robots                 = state_new_robots,
                                                    _taps                   = state_new_taps,
                                                    _total_load             = state.total_load + 1,
                                                    _total_water_available  = state.total_water_available - 1,
                                                    _non_empty_tap_cords    = state_new_non_empty_taps,
                                                    _robot_last_actions     = state_new_last_actions,
                                                    _active_robot           = index,
                                                    _is_active_only         = state.is_active_only,
                                                    _destination            = state.destination)
                                new_states.append(state_new)
                            else:
                                destinations    = state.non_satiated_plants_cords
                                if load < capacity: # this might be problematic.
                                    destinations.extend(state.non_empty_tap_cords)
                                for destination in destinations:
                                    if destination == (i,j):
                                        continue
                                    path_to_cords = self.bfs_paths[(i,j), destination]
                                    if path_to_cords is None:
                                        continue
                                    is_there_blocking_robot_in_path = False
                                    for r_cords in state.robot_cords:
                                        if r_cords != (i,j) and r_cords in path_to_cords:
                                            is_there_blocking_robot_in_path = True
                                    state_new = State(state,
                                                    _robots                 = state_new_robots,
                                                    _taps                   = state_new_taps,
                                                    _total_load             = state.total_load + 1,
                                                    _total_water_available  = state.total_water_available - 1,
                                                    _non_empty_tap_cords    = state_new_non_empty_taps,
                                                    _robot_last_actions     = state_new_last_actions,
                                                    _active_robot           = index,
                                                    _is_active_only         = not is_there_blocking_robot_in_path,
                                                    _destination            = destination)
                                    new_states.append(state_new)

                            #if self.heuristics_cache.get(state_new, None) is None:
                            moves.extend((action_name, state_new) for state_new in new_states)
                            if len(state.robots) == 1:
                                continue
                            if (len(state.non_empty_tap_cords) == 1 and state.robot_last_actions[index] == action_name) and load < min(state.plants): # and last action was LOAD, then keep LOADing
                                continue

            if is_move_legal(i-1, j):
                opposite_action = f"DOWN{{{id}}}"
                if is_there_robot(i+1, j) and not state.is_active_only:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = tuple_replace(state.robot_last_actions, index, "")
                if not is_there_robot(i-1, j) and state.robot_last_actions[index] != opposite_action:
                    if (state.is_active_only and (i-1, j) in (current_path or {})) or not state.is_active_only:
                        state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i-1,j))
                        state_new_robot_cords   = set(state_new_robot_cords_tuple)
                        action_name             = f"UP{{{id}}}"
                        state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                        state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot,
                                                        _is_active_only         = state.is_active_only,
                                                        _destination            = state.destination)

                        #if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if is_move_legal(i+1, j):
                opposite_action = f"UP{{{id}}}"
                if is_there_robot(i-1, j) and not state.is_active_only:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = tuple_replace(state.robot_last_actions, index, "")
                if not is_there_robot(i+1,j) and state.robot_last_actions[index] != opposite_action:
                    if (state.is_active_only and (i+1, j) in (current_path or {})) or not state.is_active_only:
                        state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i+1,j))
                        state_new_robot_cords   = set(state_new_robot_cords_tuple)
                        action_name             = f"DOWN{{{id}}}"
                        state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                        state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot,
                                                        _is_active_only         = state.is_active_only,
                                                        _destination            = state.destination)

                        #if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if is_move_legal(i, j-1):
                opposite_action = f"RIGHT{{{id}}}"
                if is_there_robot(i, j+1) and not state.is_active_only:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = tuple_replace(state.robot_last_actions, index, "")
                if not is_there_robot(i,j-1) and state.robot_last_actions[index] != opposite_action:
                    if (state.is_active_only and (i, j-1) in (current_path or {})) or not state.is_active_only:
                        state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i,j-1))
                        state_new_robot_cords   = set(state_new_robot_cords_tuple)
                        action_name             = f"LEFT{{{id}}}"
                        state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                        state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot,
                                                        _is_active_only         = state.is_active_only,
                                                        _destination            = state.destination)

                        #if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

            if is_move_legal(i, j+1):
                opposite_action = f"LEFT{{{id}}}"
                if is_there_robot(i, j-1) and not state.is_active_only:
                    if state.robot_last_actions[index] == opposite_action:
                        state.robot_last_actions = tuple_replace(state.robot_last_actions, index, "")
                if not is_there_robot(i,j+1) and state.robot_last_actions[index] != opposite_action:
                    if (state.is_active_only and (i, j+1) in (current_path or {})) or not state.is_active_only:
                        state_new_robot_cords_tuple = tuple_replace(state.robot_cords_tuple, index, (i,j+1))
                        state_new_robot_cords   = set(state_new_robot_cords_tuple)
                        action_name             = f"RIGHT{{{id}}}"
                        state_new_last_actions = tuple_replace(state.robot_last_actions, index, action_name)
                        state_new               = State(state, _robot_cords = state_new_robot_cords, _robot_cords_tuple = state_new_robot_cords_tuple, _robot_last_actions = state_new_last_actions, _active_robot = state.active_robot,
                                                        _is_active_only         = state.is_active_only,
                                                        _destination            = state.destination)

                        #if self.heuristics_cache.get(state_new, None) is None:
                        moves.append((action_name, state_new))

        moves = [(action_name, state_new) for (action_name, state_new) in moves if self.heuristics_cache.get(state_new, None) is None]
        return moves

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return all(not plant_water_needed for plant_water_needed in state.plants)

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        total_load                      = node.state.total_load
        total_plant_water_needed        = node.state.total_plant_water_needed
        total_water_available           = node.state.total_water_available

        # Cheaper than querying the cache...
        if not node.state.non_satiated_plants_cords:
            return 0
        if total_water_available + total_load < total_plant_water_needed:
            return float('inf')

        cached_result = self.heuristics_cache.get(node.state, None)
        if cached_result is not None:
            (value, path_cost)          = cached_result
            if path_cost < node.path_cost:
                return value
                #return float('inf')
            else:
                self.heuristics_cache[node.state] = (value, node.path_cost)
                return value
 
        min_robot_contribution_distance         = float('inf')
        for index, (id, load, capacity) in enumerate(node.state.robots):
            robot_cords = node.state.robot_cords_tuple[index]
            current_robot_contribution_distance = float('inf')
            distance_tap_then_plant = None
            if load == 0:
                if node.state.non_empty_tap_cords:
                    distance_tap_then_plant = min(
                        self.distance(tap_cords, robot_cords) + self.distance_nearest_plant(tap_cords)
                        for tap_cords in node.state.non_empty_tap_cords)
                    current_robot_contribution_distance = distance_tap_then_plant
                else:
                    current_robot_contribution_distance = float('inf')
            else:
                if total_load < total_plant_water_needed:
                    if not distance_tap_then_plant:
                        distance_tap_then_plant = min(
                        self.distance(tap_cords, robot_cords) + self.distance_nearest_plant(tap_cords)
                        for tap_cords in node.state.non_empty_tap_cords)

                    current_robot_contribution_distance  = min(
                            self.distance(plant_cords, robot_cords) + self.distance(plant_cords, tap_cords) # for some reason calculating the distance for the second plant makes it slower
                            for tap_cords       in node.state.non_empty_tap_cords
                            for plant_cords     in node.state.non_satiated_plants_cords)
                    current_robot_contribution_distance = min(distance_tap_then_plant, current_robot_contribution_distance)
                else:
                    current_robot_contribution_distance  = self.distance_nearest_plant(robot_cords)
            min_robot_contribution_distance          = min(min_robot_contribution_distance, current_robot_contribution_distance)

        heuristic                       = 2 * total_plant_water_needed - total_load

        # maybe add a clause saying that if we do not have enough water, e.g: total_load < total_plant_water_needed then we need to add the amount of water to get to a plant

        heuristic += min_robot_contribution_distance

        self.heuristics_cache[node.state] = (heuristic, node.path_cost)
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
