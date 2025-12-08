import heapq
import random
import time

GRID_SIZE = 50
OBSTACLE_DENSITY = 0.3
NUM_RUNS = 10


def make_grid(size, density):
    grid = [[0 for _ in range(size)] for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if random.random() < density:
                grid[y][x] = 1
    return grid


def get_free_pos(grid, size):
    while True:
        x, y = random.randint(0, size-1), random.randint(0, size-1)
        if grid[y][x] == 0:
            return (x, y)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star(grid, start, goal):
    size = len(grid)
    open_set = [(manhattan(start, goal), 0, start)]
    g_score = {start: 0}
    closed = set()
    nodes = 0

    t0 = time.perf_counter()

    while open_set:
        _, g, curr = heapq.heappop(open_set)

        if curr in closed:
            continue
        closed.add(curr)
        nodes += 1

        if curr == goal:
            return nodes, g, time.perf_counter() - t0

        x, y = curr
        for nx, ny in [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]:
            if 0 <= nx < size and 0 <= ny < size and grid[ny][nx] == 0:
                ng = g + 1
                if (nx, ny) not in g_score or ng < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = ng
                    heapq.heappush(
                        open_set, (ng + manhattan((nx, ny), goal), ng, (nx, ny)))

    return nodes, 0, time.perf_counter() - t0


random.seed(42)
print("MANHATTAN HEURISTIKA (4-smerový pohyb)")
print("-" * 50)

total_nodes, total_path, total_time, success = 0, 0, 0, 0

for i in range(NUM_RUNS):
    grid = make_grid(GRID_SIZE, OBSTACLE_DENSITY)
    start = get_free_pos(grid, GRID_SIZE)
    goal = get_free_pos(grid, GRID_SIZE)

    nodes, path_len, t = a_star(grid, start, goal)

    if path_len > 0:
        print(f"Beh {i+1}: uzly={nodes}, cesta={path_len}, čas={t*1000:.1f}ms")
        total_nodes += nodes
        total_path += path_len
        total_time += t
        success += 1

print("-" * 50)
print(f"PRIEMER ({success} úspešných):")
print(f"  Rozšírené uzly: {total_nodes/success:.1f}")
print(f"  Dĺžka cesty: {total_path/success:.1f}")
print(f"  Čas: {(total_time/success)*1000:.1f} ms")
