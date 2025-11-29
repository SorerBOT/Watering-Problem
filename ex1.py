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
    size:           tuple[int, int]
    walls:          dict[tuple[int, int], bool]
    taps:           dict[tuple[int, int], int]
    plants:         dict[tuple[int, int], int]
    robots:         dict[tuple[int, int], tuple[str, int, int]]
    last_actions:   list[tuple[str, str]]

    @staticmethod
    def create_initial_state(initial):
        State.size      = tuple(initial[KEY_SIZE])
        State.walls     = dict([((i,j), True) for (i,j) in initial[KEY_WALLS]]) 
        _robots         = dict(((i,j), (str(id), load, capacity)) for id, (i, j, load, capacity) in initial[KEY_ROBOTS].items())

        return State(initial[KEY_TAPS],
                     initial[KEY_PLANTS],
                     _robots)
    def __init__(self,
                    _taps: dict[tuple, int],
                    _plants: dict[tuple, int],
                    _robots: dict[tuple, tuple],
                    _last_actions: list[tuple[str, str]] = []):
        self.taps           = dict(_taps)
        self.plants         = dict(_plants)
        self.robots         = dict(_robots)
        self.last_actions   = list(_last_actions)

    def __str__(self):
        str_size            = f"Grid dimensions: {State.size}"
        str_walls           = f"Walls coordinates: {State.walls}"
        str_taps            = f"Taps: {self.taps}"
        str_plants          = f"Plants: {self.plants}"
        str_robots          = f"Robots: {self.robots}"

        return f"{str_size}\n{str_walls}\n{str_taps}\n{str_plants}\n{str_robots}"

    def __hash__(self):
        return hash((
            tuple(sorted(self.walls.items())),
            tuple(sorted(self.taps.items())),
            tuple(sorted(self.plants.items())),
            tuple(sorted(self.robots.items())),
            ))

    def __eq__(self, other):
        return (
            self.walls  == other.walls  and
            self.taps   == other.taps   and
            self.plants == other.plants and
            self.robots == other.robots)

def list_append[T](last_actions: list[T],
                 action: T):
   last_actions.append(action)

def list_replace[T](last_actions: list[T],
                   action: T):
   last_actions[:] = [action]

last_actions_update = list_replace

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    initial: State

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.initial = State.create_initial_state(initial)

    def successor(self, state: State):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        (width, height) = state.size
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

        last_action             = state.last_actions[-1] if state.last_actions else None
        for (i,j), (id, load, capacity) in state.robots.items():
            if is_move_legal(i+1, j) and last_action != ('U', id):
                state_new       = State(state.taps, state.plants, state.robots, state.last_actions)

                del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                state_new.robots[generate_robot_key(id, i+1, j, load, capacity)] = generate_robot(id, i+1, j, load, capacity)

                # need to append the hash to last_states then check for equality and prune
                last_actions_update(state_new.last_actions, ('D', id))
                moves.append((f"DOWN{{{id}}}", state_new))

            if is_move_legal(i-1, j) and last_action != ('D', id):
                state_new       = State(state.taps, state.plants, state.robots, state.last_actions)

                del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                state_new.robots[generate_robot_key(id, i-1, j, load, capacity)] = generate_robot(id, i-1, j, load, capacity)
                last_actions_update(state_new.last_actions, ('U', id))

                moves.append((f"UP{{{id}}}", state_new))

            if is_move_legal(i, j+1) and last_action != ('L', id):
                state_new       = State(state.taps, state.plants, state.robots, state.last_actions)

                del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                state_new.robots[generate_robot_key(id, i, j+1, load, capacity)] = generate_robot(id, i, j+1, load, capacity)
                last_actions_update(state_new.last_actions, ('R', id))

                moves.append((f"RIGHT{{{id}}}", state_new))

            if is_move_legal(i, j-1) and last_action != ('R', id):
                state_new       = State(state.taps, state.plants, state.robots, state.last_actions)

                del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                state_new.robots[generate_robot_key(id, i, j-1, load, capacity)] = generate_robot(id, i, j-1, load, capacity)
                state_new.last_actions.append(('L', id))
                last_actions_update(state_new.last_actions, ('L', id))

                moves.append((f"LEFT{{{id}}}", state_new))

            if (load > 0):
                plant_water_needed = state.plants.get((i,j), 0)
                if plant_water_needed > 0:
                        state_new = State(state.taps, state.plants, state.robots, state.last_actions)

                        del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                        state_new.robots[generate_robot_key(id, i, j, load - 1, capacity)] = generate_robot(id, i, j, load - 1, capacity)
                        state_new.plants[(i,j)] = plant_water_needed - 1
                        last_actions_update(state_new.last_actions, ('P', id))
                        moves.append((f"POUR{{{id}}}", state_new))

            remaining_capacity = capacity - load
            if (remaining_capacity > 0):
                water_available = state.taps.get((i,j), 0)
                if water_available > 0:
                        state_new = State(state.taps, state.plants, state.robots, state.last_actions)

                        del state_new.robots[generate_robot_key(id, i, j, load, capacity)]
                        state_new.robots[generate_robot_key(id, i, j, load + 1, capacity)] = generate_robot(id, i, j, load + 1, capacity)
                        state_new.taps[(i,j)] = water_available - 1
                        last_actions_update(state_new.last_actions, ('A', id))
                        moves.append((f"LOAD{{{id}}}", state_new))
        return moves

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return all(not v for v in state.plants.values())

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        min_distance_to_tap     = float('inf')
        min_distance_to_plant   = float('inf')
        total_load              = 0
        for (i, j), (id, load, capacity) in node.state.robots.items():
            current_distance_to_tap     = min((abs(i - tap_i) + abs(j - tap_j)) for (tap_i, tap_j) in node.state.taps.keys())
            current_distance_to_plant   = min((abs(i - plant_i) + abs(j - plant_j)) for (plant_i, plant_j) in node.state.plants.keys())
            min_distance_to_tap         = min(min_distance_to_tap, current_distance_to_tap)
            min_distance_to_plant       = min(min_distance_to_plant, current_distance_to_plant)
            total_load                  += load

        heuristic                       = (2 * sum(node.state.plants.values()) - total_load)

        if total_load == 0:
            heuristic                   += min_distance_to_tap
        else:
            heuristic                   += min(min_distance_to_tap, min_distance_to_plant)
        return heuristic

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        min_distance_to_tap     = float('inf')
        min_distance_to_plant   = float('inf')
        total_load              = 0
        for (i, j), (id, load, capacity) in node.state.robots.items():
            current_distance_to_tap     = min((abs(i - tap_i) + abs(j - tap_j)) for (tap_i, tap_j) in node.state.taps.keys())
            current_distance_to_plant   = min((abs(i - plant_i) + abs(j - plant_j)) for (plant_i, plant_j) in node.state.plants.keys())
            min_distance_to_tap         = min(min_distance_to_tap, current_distance_to_tap)
            min_distance_to_plant       = min(min_distance_to_plant, current_distance_to_plant)
            total_load                  += load

        heuristic                       = (2 * sum(node.state.plants.values()) - total_load)

        if total_load == 0:
            heuristic                   += min_distance_to_tap
        else:
            heuristic                   += min(min_distance_to_tap, min_distance_to_plant)
        return heuristic

def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
