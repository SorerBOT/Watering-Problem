import ext_plant
import ex1
import search
import numpy as np

# Set your ID here
id = ["336423982"] 

class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.game = game
        self.problem_static = game.get_problem() # specific static data like Size, Walls
        self.capacities = game.get_capacities()

    def choose_next_action(self, state):
        """ 
        Choose the next action by creating an ex1.WateringProblem from the 
        current state and running A* search to find the immediate next best move.
        """
        
        # 1. Reconstruct the problem dictionary required by ex1.WateringProblem
        #    We map the current dynamic MDP state back to the dictionary format ex1 expects.
        robots_t, plants_t, taps_t, _ = state
        
        # Reconstruct Robots dict: {id: (r, c, load, capacity)}
        robots_dict = {}
        for rid, (r, c), load in robots_t:
            robots_dict[rid] = (r, c, load, self.capacities[rid])
            
        # Reconstruct Plants dict: {(r, c): need}
        plants_dict = {pos: need for pos, need in plants_t}
        
        # Reconstruct Taps dict: {(r, c): water}
        taps_dict = {pos: water for pos, water in taps_t}
        
        # Combine static map info with current dynamic entities
        current_problem_dict = {
            "Size": self.problem_static["Size"],
            "Walls": self.problem_static["Walls"],
            "Taps": taps_dict,
            "Plants": plants_dict,
            "Robots": robots_dict
        }

        # 2. Instantiate the deterministic solver from ex1
        try:
            problem = ex1.WateringProblem(current_problem_dict)
            
            # 3. Run A* Search
            #    We use the heuristic (h_astar) provided in ex1.
            #    The search returns a Node representing the goal state.
            goal_node = search.astar_search(problem, h=problem.h_astar)
            
            if goal_node:
                # The solution() method returns the full list of actions to the goal
                plan = goal_node.solution()
                if plan:
                    # We only care about the very first step of the plan
                    next_action = plan[0]
                    
                    # 4. Format Conversion
                    #    ex1 output format: "ACTION{id}" (e.g., "UP{10}")
                    #    ex2 required format: "ACTION(id)" (e.g., "UP(10)")
                    return next_action.replace('{', '(').replace('}', ')')
                    
        except Exception:
            # If planning fails (e.g. timeout, no path found), fall through to fallback
            pass

        # Fallback: If A* fails to find a path, return a valid random move 
        # (prevents crashing if the agent gets boxed in or search fails)
        return self._random_fallback(state)

    def _random_fallback(self, state):
        """
        A copy of the random walk logic from the original starter code.
        Used only if the planner fails to find a solution.
        """
        possible_moves = []
        (robots, plants, taps, total_water_need) = state
        capacities = self.capacities
        rows, cols = self.problem_static["Size"]
        walls = self.problem_static["Walls"]

        directions = [((0, 1), "RIGHT"), ((0, -1), "LEFT"), ((-1, 0), "UP"), ((1, 0), "DOWN")]
        for robot_id, (r, c), load in robots:
            remaining_capacity = capacities[robot_id] - load
            for (dr, dc), action_name in directions:
                destination = (r + dr, c + dc)
                
                # Check grid bounds and walls
                if (destination not in walls and
                    0 <= destination[0] < rows and
                    0 <= destination[1] < cols):
                    
                    # Check if another robot is already there (simple collision avoidance)
                    occupied = False
                    for _, other_pos, _ in robots:
                        if other_pos == destination:
                            occupied = True
                            break
                    if not occupied:
                        possible_moves.append(f"{action_name}({robot_id})")
            
            if load > 0:
                # Try POUR if on a plant that needs water
                plant_in_current_location = next(((pos, need) for (pos, need) in plants if (r, c) == pos and need > 0), None)
                if plant_in_current_location is not None:
                    possible_moves.append(f"POUR({robot_id})")
            
            if remaining_capacity > 0:
                # Try LOAD if on a tap with water
                tap_in_current_location = next(((pos, available_water) for (pos, available_water) in taps if (r, c) == pos and available_water > 0), None)
                if tap_in_current_location is not None:
                    possible_moves.append(f"LOAD({robot_id})")
        
        if possible_moves:
            return np.random.choice(np.array(possible_moves))
        return "RESET"
