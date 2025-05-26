"""
run_maps.py – Visualizer & tester for battery-constrained map exploration.
Maps are generated using map_surface_with_battery_constraint(), where the rover
has a limited battery (default 20 units). This script reports:
- Visual grid output
- Map dimensions
- Coverage percentage
- Unreachable `?` tiles that cannot be accessed even with unlimited energy
"""

from planet_intel import PlanetIntel
from main import map_surface_with_battery_constraint
from collections import deque


def pretty(grid: list[list[str]]) -> str:
    """Return the 2-D list as newline-separated strings."""
    return '\n'.join(''.join(row) for row in grid)


def reachable(home, target, planet):
    """Check if `target` is reachable from `home` using BFS on planet._Planet__grid."""
    grid = planet._Planet__grid
    max_r = len(grid)
    max_c = len(grid[0]) if max_r > 0 else 0

    Q = deque([home])
    seen = {home}

    while Q:
        r, c = Q.popleft()
        for dr, dc in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < max_r and 0 <= nc < max_c):
                continue
            if (nr, nc) == target:
                return True
            if (nr, nc) in seen or grid[nr][nc] == 'X':
                continue
            seen.add((nr, nc))
            Q.append((nr, nc))

    return False


def run_map_test(name: str, planet):
    print(f"\n=== {name} — Battery-Constrained Exploration ===")
    explored = map_surface_with_battery_constraint(planet)
    print(pretty(explored))

    rows, cols = len(explored), len(explored[0])
    print(f"\nDimensions: {rows} rows × {cols} cols")

    total = sum(len(row) for row in explored)
    known = sum(cell in ('.', 'w', 'H', 'X') for row in explored for cell in row)
    coverage = 100 * known / total if total else 0
    print(f"Coverage: {known}/{total} = {coverage:.1f}%")

    print("Unreachable `?` tiles (confirmed unreachable even with infinite energy):")
    planet_grid = planet._Planet__grid
    home_pos = next((r, c)
                    for r, row in enumerate(planet_grid)
                    for c, ch in enumerate(row) if ch == 'H')

    for r in range(rows):
        for c in range(cols):
            if explored[r][c] == '?' and not reachable(home_pos, (r, c), planet):
                print(f"  - ({r}, {c}) is unreachable")


def main():
    official_planets = [
        ("Planet 1", PlanetIntel.get_planet_1()),
        ("Planet 2", PlanetIntel.get_planet_2()),
        ("Planet 3", PlanetIntel.get_planet_3()),
    ]

    custom_maps = [
        ("Bottleneck Maze", PlanetIntel.create_test_planet([
            "H...X....",
            ".XXX.X.X.",
            ".....X...",
            "X.X.X.XXX",
            "...X....."
        ])),
        ("Open Field Far Edges", PlanetIntel.create_test_planet([
            "H........",
            ".........",
            "..XXX....",
            ".........",
            ".......X.",
            ".....w..X",
            "....X...."
        ])),
        ("Tight Corridors", PlanetIntel.create_test_planet([
            "H.X.X.X.",
            ".X.X.X.X",
            ".X.X.X.X",
            ".X.X.X.X",
            ".......X"
        ]))
    ]

    for name, planet in official_planets + custom_maps:
        run_map_test(name, planet)


if __name__ == "__main__":
    main()

