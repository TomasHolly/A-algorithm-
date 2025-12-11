import heapq
import random
import time
import math

GRID_SIZE = 50
OBSTACLE_DENSITY = 0.3
NUM_RUNS = 10


def make_grid(size, density):
    """Vytvorí mriežku s náhodnými prekážkami"""
    grid = [[0 for _ in range(size)] for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if random.random() < density:
                grid[y][x] = 1
    return grid


def get_free_position(grid, size):
    """Nájde náhodnú voľnú pozíciu"""
    while True:
        x = random.randint(0, size - 1)
        y = random.randint(0, size - 1)
        if grid[y][x] == 0:
            return (x, y)


def euclidean_heuristic(a, b):
    """Euklidovská vzdialenosť"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def a_star(grid, start, goal, heuristic):
    """A* algoritmus s 4-smerovým pohybom"""
    size = len(grid)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))

    came_from = {}
    g_score = {start: 0}
    closed_set = set()
    nodes_expanded = 0

    start_time = time.perf_counter()

    while open_set:
        f, g, current = heapq.heappop(open_set)

        if current in closed_set:
            continue

        closed_set.add(current)
        nodes_expanded += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()

            path_length = len(path) - 1
            time_taken = time.perf_counter() - start_time

            return {
                'success': True,
                'nodes_expanded': nodes_expanded,
                'path_length': path_length,
                'time': time_taken
            }

        x, y = current
        # IBA 4 SMERY - bez diagonál
        neighbors = [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1)
        ]

        for nx, ny in neighbors:
            if 0 <= nx < size and 0 <= ny < size and grid[ny][nx] == 0:
                neighbor = (nx, ny)
                tentative_g = g_score[current] + 1  # jednotkový krok

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current

    time_taken = time.perf_counter() - start_time
    return {
        'success': False,
        'nodes_expanded': nodes_expanded,
        'path_length': 0,
        'time': time_taken
    }


def main():
    print("=" * 60)
    print("A* ALGORITMUS - EUKLIDOVSKÁ HEURISTIKA (4-SMEROVÝ POHYB)")
    print("=" * 60)
    print(f"Veľkosť mriežky: {GRID_SIZE}x{GRID_SIZE}")
    print(f"Hustota prekážok: {OBSTACLE_DENSITY * 100}%")
    print(f"Počet behov: {NUM_RUNS}")
    print(f"Typ pohybu: 4-smerový (IBA hore/dole/vľavo/vpravo)")
    print("=" * 60)

    total_nodes = 0
    total_path = 0
    total_time = 0
    successful_runs = 0

    for run in range(1, NUM_RUNS + 1):
        grid = make_grid(GRID_SIZE, OBSTACLE_DENSITY)
        start = get_free_position(grid, GRID_SIZE)
        goal = get_free_position(grid, GRID_SIZE)

        result = a_star(grid, start, goal, euclidean_heuristic)

        print(f"\nBeh {run}/{NUM_RUNS}:")
        print(f"  Start: {start}, Cieľ: {goal}")

        if result['success']:
            print(f"  ✓ Cesta nájdená")
            print(f"  - Rozšírené uzly: {result['nodes_expanded']}")
            print(f"  - Dĺžka cesty: {result['path_length']}")
            print(f"  - Čas: {result['time']*1000:.2f} ms")

            total_nodes += result['nodes_expanded']
            total_path += result['path_length']
            total_time += result['time']
            successful_runs += 1
        else:
            print(f"  ✗ Cesta nenájdená")

    print("\n" + "=" * 60)
    print("VÝSLEDKY")
    print("=" * 60)
    print(f"Úspešné behy: {successful_runs}/{NUM_RUNS}")

    if successful_runs > 0:
        print(f"\nPRIEMERNÉ HODNOTY:")
        print(
            f"  Počet rozšírených uzlov: {total_nodes / successful_runs:.1f}")
        print(f"  Dĺžka cesty: {total_path / successful_runs:.1f}")
        print(f"  Čas výpočtu: {(total_time / successful_runs)*1000:.2f} ms")

    print("=" * 60)


if __name__ == "__main__":
    random.seed(42)
    main()
