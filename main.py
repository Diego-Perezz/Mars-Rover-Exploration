import math
from planet_intel import PlanetIntel
from rover import Rover

# ──────────────────────────────────────────────────────────────
# Checkpoint Part – DFS-based surface mapping (no battery limit)
# ──────────────────────────────────────────────────────────────

def map_surface(planet):
    """
    Explores the entire surface of the planet using DFS with unlimited battery.

    Parameters:
        planet (Planet): The planet object to explore.

    Returns:
        list[list[str]]: A dense 2D list representing the explored terrain map.
    """
    rover = Rover(planet, math.inf)
    explored = {}
    visited = set()

    DIRS = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
    REVERSE = {d: {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[d] for d in DIRS}
    grid = planet._Planet__grid
    num_rows, num_cols = len(grid), len(grid[0])

    def in_bounds(r, c):
        return 0 <= r < num_rows and 0 <= c < num_cols

    def dfs(r, c):
        """
        Recursive DFS that explores all reachable tiles from (r, c).
        """
        if (r, c) in visited:
            return
        visited.add((r, c))
        explored[(r, c)] = rover.get_current_location()
        for d, (dr, dc) in DIRS.items():
            nr, nc = r + dr, c + dc
            success, _ = rover.move(d)
            if success:
                dfs(nr, nc)
                rover.move(REVERSE[d])
            elif in_bounds(nr, nc) and (nr, nc) not in explored:
                explored[(nr, nc)] = 'X'

    start = next((r, c) for r in range(num_rows) for c in range(num_cols) if grid[r][c] == 'H')
    dfs(*start)
    return _sparse_to_dense(explored)




# ──────────────────────────────────────────────────────────────
# Battery-Constrained Exploration – BFS-based layered expansion
# ──────────────────────────────────────────────────────────────

FULL_BATTERY = 20
DIRS = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
OPPOSITE = {d: {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[d] for d in DIRS}

def map_surface_with_battery_constraint(planet):
    """
    Explores as much of the planet as possible within a 20-unit battery range using BFS.

    Parameters:
        planet (Planet): The planet object to explore.

    Returns:
        list[list[str]]: A dense 2D list representing the explored terrain map.
    """
    rover = Rover(planet)
    home = (0, 0)
    explored = {home: rover.get_current_location()}
    parents = {home: None}
    frontier = [home]
    radius = 0

    while frontier and 2 * (radius + 1) <= FULL_BATTERY:
        next_frontier = []
        for tile in frontier:
            path = _reconstruct_path(tile, parents)
            _travel(rover, path)

            for d, nbr in _neighbours(tile):
                if nbr in explored:
                    continue
                if _roundtrip_cost(path, nbr, home) > FULL_BATTERY:
                    continue
                success, reason = rover.move(d)
                if not success:
                    if reason == "Obstructed Space":
                        explored[nbr] = 'X'
                    continue
                explored[nbr] = rover.get_current_location()
                parents[nbr] = tile
                next_frontier.append(nbr)
                rover.move(OPPOSITE[d])  # step back to parent 

            _travel(rover, _reverse_path(path))  # return to home 

        frontier = next_frontier
        radius += 1

    return _sparse_to_dense(explored)

# ──────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────

def _neighbours(pos):
    """
    Yields neighboring coordinates for a given position.

    Parameters:
        pos (tuple[int, int]): Current (row, col) position.

    Yields:
        tuple[str, tuple[int, int]]: Direction and neighbor coordinate.
    """
    r, c = pos
    for d, (dr, dc) in DIRS.items():
        yield d, (r + dr, c + dc)

def _reconstruct_path(target, parents):
    """
    Reconstructs the path from home to a given target using the parents dictionary.

    Parameters:
        target (tuple[int, int]): The target coordinate.
        parents (dict): A mapping of node to its parent.

    Returns:
        list[str]: List of directions from home to the target.
    """
    path = []
    cur = target
    while parents[cur]:
        prev = parents[cur]
        for d, (dr, dc) in DIRS.items():
            if (prev[0] + dr, prev[1] + dc) == cur:
                path.append(d)
                break
        cur = prev
    return list(reversed(path))

def _reverse_path(path):
    """
    Returns the reverse of a given path.

    Parameters:
        path (list[str]): A list of directions.

    Returns:
        list[str]: The reversed path with opposite directions.
    """
    return [OPPOSITE[d] for d in reversed(path)]

def _travel(rover, path):
    """
    Moves the rover along a sequence of directions.

    Parameters:
        rover (Rover): The rover instance.
        path (list[str]): A list of directions to move.
    """
    for d in path:
        rover.move(d)

def _roundtrip_cost(path_to_tile, nbr, home):
    """
    Estimates the cost of reaching a neighbor and returning to home.

    Parameters:
        path_to_tile (list[str]): Path to the current tile.
        nbr (tuple[int, int]): Neighbor coordinate.
        home (tuple[int, int]): Home coordinate.

    Returns:
        int: Estimated round-trip cost.
    """
    return len(path_to_tile) + 1 + _manhattan(nbr, home)

def _manhattan(a, b):
    """
    Calculates the Manhattan distance between two points.

    Parameters:
        a (tuple[int, int]): First point.
        b (tuple[int, int]): Second point.

    Returns:
        int: Manhattan distance.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def _sparse_to_dense(explored):
    """
    Converts a sparse dict of explored tiles to a dense 2D list.

    Parameters:
        explored (dict): {(row, col): char} mapping.

    Returns:
        list[list[str]]: Dense 2D list map.
    """
    in_bounds = [pos for pos, ch in explored.items() if ch != 'X']
    if not in_bounds:
        return [[]]
    rows, cols = zip(*in_bounds)
    min_r, max_r = min(rows), max(rows)
    min_c, max_c = min(cols), max(cols)
    h, w = max_r - min_r + 1, max_c - min_c + 1
    grid = [['?' for _ in range(w)] for _ in range(h)]
    for (r, c), ch in explored.items():
        rr, cc = r - min_r, c - min_c
        if 0 <= rr < h and 0 <= cc < w:
            grid[rr][cc] = ch
    return _trim_unknown_border(grid)

def _trim_unknown_border(grid):
    """
    Trims rows and columns that contain only '?' from the grid edges.

    Parameters:
        grid (list[list[str]]): Dense 2D list map.

    Returns:
        list[list[str]]: Trimmed 2D list map.
    """
    while grid and all(ch == '?' for ch in grid[0]): grid.pop(0)
    while grid and all(ch == '?' for ch in grid[-1]): grid.pop()
    if not grid: return [[]]
    transposed = list(map(list, zip(*grid)))
    while transposed and all(ch == '?' for ch in transposed[0]): transposed.pop(0)
    while transposed and all(ch == '?' for ch in transposed[-1]): transposed.pop()
    return [list(row) for row in zip(*transposed)]

def print_map_to_file(map_data, filename):
    """
    Writes a 2D terrain map to a file.

    Parameters:
        map_data (list[list[str]]): The terrain map to write.
        filename (str): The file path to write to.
    """
    with open(filename, 'w') as f:
        for row in map_data:
            f.write(''.join(row) + '\n')
