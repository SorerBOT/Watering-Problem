import ex1_check
import search
import utils
from collections import deque

id = ["213209174"]  

class WateringProblem(search.Problem):
    """Watering problem with BFS precomputation and light pruning."""
    heuristic_invocations = 0

    INF = 10**9  # class-level constant, reused everywhere

    def __init__(self, initial):
        # ----- Static grid -----
        self.grid_size = tuple(initial["Size"])
        self.N, self.M = self.grid_size
        self.grid_walls = frozenset(initial.get("Walls", set()))

        # ----- Cell degrees (for dead-end pruning) -----
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        walls = self.grid_walls
        N, M = self.N, self.M

        cell_degree = {}
        for i in range(N):
            for j in range(M):
                if (i, j) in walls:
                    continue
                deg = 0
                for di, dj in dirs:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < N and 0 <= nj < M and (ni, nj) not in walls:
                        deg += 1
                cell_degree[(i, j)] = deg
        self.cell_degree = cell_degree

        # ----- Precompute legal base moves for each cell -----
        self.base_moves = {}   # (i,j) → list of (dir, ni, nj)

        dirs2 = {
            "UP":    (-1, 0),
            "DOWN":  (1, 0),
            "LEFT":  (0, -1),
            "RIGHT": (0, 1)
        }

        for i in range(self.N):
            for j in range(self.M):
                if (i, j) in self.grid_walls:
                    continue
                moves = []
                for dname, (di, dj) in dirs2.items():
                    ni, nj = i + di, j + dj
                    if 0 <= ni < self.N and 0 <= nj < self.M:
                        if (ni, nj) not in self.grid_walls:
                            moves.append((dname, ni, nj))
                self.base_moves[(i, j)] = moves


        # Static entities: plants, robots, taps 
        self.plant_targets = dict(initial.get("Plants", {}))
        self.robot_capacities = {
            rid: capacity
            for rid, (r, c, load, capacity) in initial.get("Robots", {}).items()
        }

        self.tap_positions = list(initial.get("Taps", {}).keys())
        self.plant_positions = list(self.plant_targets.keys())

        # Small-case flags for pruning
        self.one_robot = (len(initial.get("Robots", {})) == 1)
        self.one_tap   = (len(self.tap_positions) == 1)
        self.one_plant = (len(self.plant_positions) == 1)

        # ----- Precompute distances -----
        self._precompute_distances()

        # ----- Build initial state -----
        robots = tuple(sorted(
            (rid, r, c, load)
            for rid, (r, c, load, _cap) in initial.get("Robots", {}).items()
        ))

        taps = tuple(sorted(
            (pos, water)
            for pos, water in initial.get("Taps", {}).items()
        ))

        plants = tuple(sorted(
            (pos, need)
            for pos, need in initial.get("Plants", {}).items()
        ))

        # remaining: total water still needed (for cheap goal_test)
        remaining = sum(need for (_pos, need) in plants)

        initial_state = (robots, taps, plants, remaining)
        search.Problem.__init__(self, initial_state)

    # ------------------------------------------------------------------
    # BFS helpers
    # ------------------------------------------------------------------

    def _multi_source_bfs(self, sources):
        """
        Multi-source BFS on grid: distance from every cell to the nearest source.
        """
        N, M = self.N, self.M
        walls = self.grid_walls
        INF = self.INF

        dist = [[INF] * M for _ in range(N)]
        if not sources:
            return dist

        q = deque()
        for (si, sj) in sources:
            if 0 <= si < N and 0 <= sj < M and (si, sj) not in walls:
                dist[si][sj] = 0
                q.append((si, sj))

        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while q:
            i, j = q.popleft()
            d0 = dist[i][j]
            nd = d0 + 1
            for di, dj in dirs:
                ni, nj = i + di, j + dj
                if 0 <= ni < N and 0 <= nj < M and (ni, nj) not in walls:
                    if dist[ni][nj] > nd:
                        dist[ni][nj] = nd
                        q.append((ni, nj))

        return dist

    def _bfs_from_single_source(self, start):
        """
        Single-source BFS from 'start' to all cells.
        Used per plant.
        """
        N, M = self.N, self.M
        walls = self.grid_walls
        INF = self.INF

        dist = [[INF] * M for _ in range(N)]
        si, sj = start
        if (si, sj) in walls:
            return dist

        dist[si][sj] = 0
        q = deque([start])

        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while q:
            i, j = q.popleft()
            d0 = dist[i][j]
            nd = d0 + 1
            for di, dj in dirs:
                ni, nj = i + di, j + dj
                if 0 <= ni < N and 0 <= nj < M and (ni, nj) not in walls:
                    if dist[ni][nj] > nd:
                        dist[ni][nj] = nd
                        q.append((ni, nj))

        return dist

    def _precompute_distances(self):
        """
        Precompute:
        - dist_to_tap[i][j] : distance from (i,j) to nearest tap.
        - dist_from_plant[p][i][j] : distance from plant p to (i,j).
        - plant_min_tap_dist[p] : min distance from plant p to any tap
                                  (used in heuristic, computed once).
        """
        INF = self.INF

        # distance to nearest tap (multi-source BFS)
        self.dist_to_tap = self._multi_source_bfs(self.tap_positions)

        # distance from each plant to all cells, and plant->nearest tap
        self.dist_from_plant = {}
        self.plant_min_tap_dist = {}

        for p in self.plant_positions:
            dist_p = self._bfs_from_single_source(p)
            self.dist_from_plant[p] = dist_p

            # Min distance from this plant to any tap (lower bound)
            min_tap = INF
            for (tx, ty) in self.tap_positions:
                if 0 <= tx < self.N and 0 <= ty < self.M:
                    d_tp = dist_p[tx][ty]
                    if d_tp < min_tap:
                        min_tap = d_tp
            self.plant_min_tap_dist[p] = min_tap

   
    def successor(self, state):
        """
        Generate all legal actions from the current state.
        State: (robots, taps, plants, remaining)
        robots: (rid, r, c, load)
        taps:   (pos, water_remaining)
        plants: (pos, need_remaining)
        remaining: sum of all plant needs
        """
        robots, taps, plants, remaining = state
        if remaining == 0:
            return []

        # Quick dicts (O(#plants + #taps))
        tap_dict = dict(taps)
        plant_dict = dict(plants)

        # Robot positions for collision checks
        robot_positions = {(r, c) for (_rid, r, c, _load) in robots}

        # Non-empty plants / taps (because we remove empty ones)
        needy_plants = {pos for (pos, _need) in plants}
        active_taps = {pos for (pos, _amt) in taps}

        walls = self.grid_walls
        successors = []

        one_tap   = self.one_tap
        one_plant = self.one_plant
        one_robot = self.one_robot
        dist_to_tap = self.dist_to_tap
        dist_from_plant = self.dist_from_plant
        cell_degree = self.cell_degree
        tap_positions = set(self.tap_positions)  # for quick membership

        for rid, r, c, load in robots:
            capacity = self.robot_capacities[rid]

            # ---------- ONE-ROBOT SPECIAL CASES ----------
            force_stay_on_tap = False
            force_pour_here = False

            # (1) One robot + standing on a tap that has water + we can still use more load
            #     → stay here and LOAD instead of moving away.
            if one_robot and (r, c) in active_taps and remaining > 0:
                tap_amt = tap_dict.get((r, c), 0)
                if tap_amt > 0:
                    max_useful_load = min(capacity, remaining, tap_amt)
                    if load < max_useful_load:
                        force_stay_on_tap = True

            # (2) One robot + standing on ANY needy plant with water (and it's not also a tap)
            #     → always safe to POUR now instead of walking away.
            if one_robot and load > 0 and (r, c) in plant_dict:
                need_here = plant_dict[(r, c)]
                if need_here > 0 and (r, c) not in active_taps:
                    force_pour_here = True

            # ---------- LOAD ----------
            if (r, c) in active_taps and load < capacity:
                new_robots = []
                for (robot_id, rr, rc, ld) in robots:
                    if robot_id == rid:
                        new_robots.append((robot_id, r, c, load + 1))
                    else:
                        new_robots.append((robot_id, rr, rc, ld))
                new_robots = tuple(new_robots)

                new_taps = []
                for (pos, amt) in taps:
                    if pos == (r, c):
                        new_amt = amt - 1
                        # remove tap if it becomes empty
                        if new_amt > 0:
                            new_taps.append((pos, new_amt))
                    else:
                        new_taps.append((pos, amt))
                new_taps = tuple(new_taps)

                successors.append((
                    f"LOAD{{{rid}}}",
                    (new_robots, new_taps, plants, remaining)
                ))

            # ---------- POUR ----------
            if (r, c) in needy_plants and load > 0:
                new_robots = []
                for (robot_id, rr, rc, ld) in robots:
                    if robot_id == rid:
                        new_robots.append((robot_id, r, c, load - 1))
                    else:
                        new_robots.append((robot_id, rr, rc, ld))
                new_robots = tuple(new_robots)

                new_plants_list = []
                for (pos, need) in plants:
                    if pos == (r, c):
                        new_need = need - 1
                        # remove plant if it becomes fully satisfied
                        if new_need > 0:
                            new_plants_list.append((pos, new_need))
                    else:
                        new_plants_list.append((pos, need))
                new_plants = tuple(new_plants_list)

                new_remaining = remaining - 1

                successors.append((
                    f"POUR{{{rid}}}",
                    (new_robots, taps, new_plants, new_remaining)
                ))

            # One-robot: if we decided we should just keep loading here, don't move.
            if force_stay_on_tap:
                continue

            # One-robot: if we decided we should just pour here, don't move.
            if force_pour_here:
                continue

            # ---------- MOVES ----------
            for direction, nr, nc in self.base_moves[(r, c)]:
                # Collision with other robots
                if (nr, nc) in robot_positions:
                    continue

                # open-grid Manhattan pruning (no walls case)
                if len(walls) == 0:
                    if load == 0 and active_taps:
                        best_curr = min(abs(r - tx) + abs(c - ty)
                                        for (tx, ty) in active_taps)
                        best_next = min(abs(nr - tx) + abs(nc - ty)
                                        for (tx, ty) in active_taps)
                        if best_next > best_curr:
                            continue

                    if load > 0 and needy_plants:
                        best_curr = min(abs(r - px) + abs(c - py)
                                        for (px, py) in needy_plants)
                        best_next = min(abs(nr - px) + abs(nc - py)
                                        for (px, py) in needy_plants)
                        if best_next > best_curr:
                            continue

                # one-tap generic pruning: if empty, move closer to *some* tap
                if one_tap and active_taps and load == 0:
                    if dist_to_tap[nr][nc] > dist_to_tap[r][c]:
                        continue

                # PRUNE C: empty robot into dead-end satisfied plant
                if load == 0:
                    need = plant_dict.get((nr, nc), None)
                    if need is not None and need == 0:
                        if (nr, nc) not in tap_positions:
                            if cell_degree.get((nr, nc), 0) <= 1:
                                continue

                # Safe: one plant + NO TAP WATER + robot loaded → robot must approach plant
                if one_plant and load > 0 and not active_taps:
                    plant_pos = self.plant_positions[0]
                    dist_p = dist_from_plant[plant_pos]
                    d_curr_p = dist_p[r][c]
                    d_next_p = dist_p[nr][nc]
                    if d_next_p > d_curr_p:
                        continue

                # Gentle prune: one robot, avoid running too far from taps
                if one_robot and load == 0 and active_taps:
                    d_curr = dist_to_tap[r][c]
                    d_next = dist_to_tap[nr][nc]
                    if d_next > d_curr + 1:
                        continue

                # Build successor state for this move
                new_robots = []
                for (robot_id, rr, rc, ld) in robots:
                    if robot_id == rid:
                        new_robots.append((robot_id, nr, nc, load))
                    else:
                        new_robots.append((robot_id, rr, rc, ld))
                new_robots = tuple(new_robots)

                successors.append((
                    f"{direction}{{{rid}}}",
                    (new_robots, taps, plants, remaining)
                ))

        return successors


    # ------------------------------------------------------------------
    # Goal test
    # ------------------------------------------------------------------

    def goal_test(self, state):
        """We track remaining water directly."""
        robots, taps, plants, remaining = state
        if remaining == 0:
            print(f"Heuristic invocations: {self.heuristic_invocations}")
            return True

    # ------------------------------------------------------------------
    # A* heuristic (admissible, BFS-based)
    # ------------------------------------------------------------------

    def h_astar(self, node):
        """
        Admissible A* heuristic:

        h = remaining_pours
          + remaining_loads
          + movement_lower_bound

        where:
        - remaining_pours = total remaining water
        - remaining_loads = extra water we must still load from taps
        - movement_lower_bound: for each needy plant p, we compute how cheaply
          some robot could help it (robot->plant if loaded, or robot->tap->plant
          using precomputed plant->nearest-tap distance) and take the max over plants.
        """
        self.heuristic_invocations += 1
        state = node.state
        robots, taps, plants, remaining = state

        INF = self.INF

        # Already solved?
        remaining_pours = remaining
        if remaining_pours == 0:
            return 0

        # Action-cost lower bound
        robot_water = sum(load for (_rid, _r, _c, load) in robots)
        pours_lb = remaining_pours                     # each unit must be poured once
        loads_lb = max(0, remaining_pours - robot_water)  # units not yet on robots must be loaded
        action_cost = pours_lb + loads_lb

        # Movement lower bound
        needy_plants = [(pos, need) for (pos, need) in plants if need > 0]

        movement_lb = 0
        dist_to_tap = self.dist_to_tap
        dist_from_plant = self.dist_from_plant
        plant_min_tap_dist = self.plant_min_tap_dist

        for (p_pos, need) in needy_plants:
            dist_p = dist_from_plant[p_pos]
            best_for_p = INF

            # precomputed lower bound tap->plant
            min_tap_to_p = plant_min_tap_dist[p_pos]

            for (_rid, rx, ry, load) in robots:
                if not (0 <= rx < self.N and 0 <= ry < self.M):
                    continue

                if load > 0:
                    # Robot already has water: just robot->plant distance
                    d_rp = dist_p[rx][ry]
                    if d_rp < best_for_p:
                        best_for_p = d_rp
                else:
                    # Robot empty: robot->nearest tap + tap->plant (lower bound using precomputed min_tap_to_p)
                    d_rt = dist_to_tap[rx][ry]
                    if d_rt < INF and min_tap_to_p < INF:
                        cand = d_rt + min_tap_to_p
                        if cand < best_for_p:
                            best_for_p = cand

            if best_for_p < INF and best_for_p > movement_lb:
                movement_lb = best_for_p

        if movement_lb == INF:
            movement_lb = 0  # conservative fallback

        return action_cost + movement_lb

    # ------------------------------------------------------------------
    # GBFS heuristic
    # ------------------------------------------------------------------

    def h_gbfs(self, node):
        """
        Fast GBFS heuristic - reuses the existing distance grids.
        """

        state = node.state
        robots, taps, plants, remaining = state

        if remaining == 0:
            return 0

        # Base: pour + load lower bound
        robot_water = sum(load for (_rid, _r, _c, load) in robots)
        base = remaining + max(0, remaining - robot_water)

        # Quick urgency factor: average distance from robots to first few needy plants
        needy = [pos for (pos, need) in plants if need > 0]
        if not needy:
            return base

        total_dist = 0
        checked = 0

        for p_pos in needy[:3]:  # only first few plants to keep it fast
            dist_p = self.dist_from_plant[p_pos]
            # distance from nearest robot to this plant
            min_d = min(dist_p[rx][ry] for (_rid, rx, ry, _load) in robots)
            total_dist += min_d
            checked += 1

        avg_dist = total_dist // max(1, checked) if checked > 0 else 0

        return base + avg_dist


def create_watering_problem(game):
    print("<<create_watering_problem")
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
