# Grid-based A* pathfinding visualization
# The map layout mirrors the data from Astar.asm

import heapq
from typing import List, Tuple, Dict, Optional

# Map with different garden objects. Characters other than '.', 'S', 'G'
# are treated as obstacles for the pathfinding algorithm.
MAP_ROWS: List[str] = [
    "S...B..........G",
    "................",
    "..PPPP..........",
    "..P..P..........",
    "..P..P...F......",
    "................",
    ".......B........",
    ".......T........",
    ".......F........",
    ".......RR.......",
    "..C.............",
    "................",
    "................",
    "................",
    "................",
    "..............G",
]

# Color definitions for pretty terminal output
COLOR_MAP: Dict[str, Tuple[int, int, int]] = {
    'S': (255, 255, 255),  # white
    'G': (255, 255, 0),    # yellow
    'B': (0, 128, 0),      # dark green
    'T': (139, 69, 19),    # brown
    'P': (0, 0, 255),      # blue
    'F': (255, 0, 255),    # magenta
    'R': (128, 128, 128),  # gray
    'C': (165, 42, 42),    # brown
    '#': (255, 0, 0),      # red
    '.': (0, 255, 0),      # grass
    'X': (0, 0, 0),        # path
}

# Characters that can be traversed by the pathfinder
WALKABLE = {'.', 'S', 'G'}

Grid = List[List[str]]
Point = Tuple[int, int]


def parse_grid(rows: List[str]) -> Grid:
    """Convert list of strings to list of lists for mutability."""
    return [list(row) for row in rows]


def find_char(grid: Grid, ch: str) -> List[Point]:
    """Return all coordinates of a given character in the grid."""
    coords = []
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == ch:
                coords.append((r, c))
    return coords


def neighbors(grid: Grid, point: Point) -> List[Point]:
    """Return 4-connected neighbors within bounds."""
    r, c = point
    deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    result = []
    for dr, dc in deltas:
        nr, nc = r + dr, c + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
            result.append((nr, nc))
    return result


def heuristic(a: Point, b: Point) -> int:
    """Manhattan distance heuristic used by A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid: Grid, start: Point, goal: Point) -> Optional[List[Point]]:
    """Perform A* search on the grid."""
    open_set: List[Tuple[int, int, Point]] = []  # (f, g, point)
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))

    came_from: Dict[Point, Point] = {}
    g_score: Dict[Point, int] = {start: 0}
    visited: set[Point] = set()

    while open_set:
        f, g, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            # Goal reached; reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for n in neighbors(grid, current):
            if grid[n[0]][n[1]] not in WALKABLE:
                continue
            tentative_g = g + 1
            if tentative_g < g_score.get(n, float('inf')):
                g_score[n] = tentative_g
                came_from[n] = current
                f_score = tentative_g + heuristic(n, goal)
                heapq.heappush(open_set, (f_score, tentative_g, n))

    return None  # No path found


def overlay_path(grid: Grid, path: List[Point]) -> Grid:
    """Return a copy of the grid with path drawn using 'X'."""
    new_grid = [row.copy() for row in grid]
    for r, c in path:
        if new_grid[r][c] in {'S', 'G'}:
            continue
        new_grid[r][c] = 'X'
    return new_grid


def colorize(ch: str) -> str:
    """Return ANSI colored representation of a cell."""
    r, g, b = COLOR_MAP.get(ch, (0, 255, 0))
    return f"\033[48;2;{r};{g};{b}m {ch} \033[0m"


def display(grid: Grid) -> None:
    """Print the grid with ANSI colors."""
    for row in grid:
        print(''.join(colorize(c) for c in row))


def main() -> None:
    grid = parse_grid(MAP_ROWS)
    start = find_char(grid, 'S')[0]
    goals = find_char(grid, 'G')
    goal = goals[-1]  # choose the last goal (bottom-right corner)

    path = astar(grid, start, goal)
    if path is None:
        print("No path found")
        display(grid)
        return

    print(f"Path length: {len(path)}")
    grid_with_path = overlay_path(grid, path)
    display(grid_with_path)


if __name__ == "__main__":
    main()
