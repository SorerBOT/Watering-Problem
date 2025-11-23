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
    size        = (0,0) 
    walls       = frozenset()
    taps:       dict[tuple, int]
    plants:     dict[tuple, int]
    robots:     dict[int, tuple]

    @staticmethod
    def create_initial_state(initial):
        State.size       = tuple(initial[KEY_SIZE])
        State.walls      = frozenset(initial[KEY_WALLS])

        return State(initial[KEY_TAPS],
                     initial[KEY_PLANTS],
                     initial[KEY_ROBOTS])
        
    def __init__(self, taps, plants, robots):
        self.taps       = dict(taps)
        self.plants     = dict(plants)
        self.robots     = dict(robots)

    def __str__(self):
        str_size    = f"Grid dimensions: {State.size}"
        str_walls   = f"Walls coordinates: {State.walls}"
        str_taps    = f"Taps: {self.taps}"
        str_plants    = f"Plants: {self.plants}"
        str_robots    = f"Robots: {self.robots}"

        return f"{str_size}\n{str_walls}\n{str_taps}\n{str_plants}\n{str_robots}"

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""
    state: State

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)
        self.state = State.create_initial_state(initial)
        print ( str(self.state) )

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        utils.raiseNotDefined()

    def goal_test(self, state: State):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        return state.plants

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        utils.raiseNotDefined()

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        utils.raiseNotDefined()


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
