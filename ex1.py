import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]


class State:
    KEY_SIZE    = "Size"
    KEY_WALLS   = "Walls"
    KEY_TAPS    = "Taps"
    KEY_PLANTS  = "Plants"
    KEY_ROBOTS  = "Robots"
    size        = (0,0) 
    walls       = frozenset()

    @staticmethod
    def create_initial_state(initial):
        State.size       = initial[State.KEY_SIZE]
        State.walls      = frozenset(initial[State.KEY_WALLS])

        return State(initial[State.KEY_TAPS],
                     initial[State.KEY_PLANTS],
                     initial[State.KEY_ROBOTS])
        
    def __init__(self, taps, plants, robots):
        self.taps       = taps
        self.plants     = plants
        self.robots     = robots

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        utils.raiseNotDefined()

    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        utils.raiseNotDefined()

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
