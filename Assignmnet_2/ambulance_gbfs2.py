import heapq
import matplotlib.pyplot as plt

# Map Kota
city_map = [
    ['S', '.', '.', 'T', '.', '.', '.'],
    ['.', 'T', '.', '.', '.', 'T', '.'],
    ['.', 'T', '.', 'T', '.', '.', '.'],
    ['.', '.', '.', 'T', '.', 'T', '.'],
    ['T', 'T', '.', '.', '.', '.', 'H']
]

# Ukuran grid
rows = len(city_map)
cols = len(city_map[0])

# Cari posisi Start (S) dan Hospital (H)
def find_position(symbol):
    for r in range(rows):
        for c in range(cols):
            if city_map[r][c] == symbol:
                return (r, c)
    return None

start = find_position('S')
goal = find_position('H')

# Heuristic: Manhattan Distance
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

# Cek apakah posisi valid
def valid(r, c):
    return 0 <= r < rows and 0 <= c < cols and city_map[r][c] != 'T'

# GBFS Algorithm
def gbfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {}
    visited = set()
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        visited.add(current)
        
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0] + dr, current[1] + dc)
            
            if valid(*neighbor) and neighbor not in visited:
                heapq.heappush(open_set, (heuristic(neighbor, goal), neighbor))
                if neighbor not in came_from:
                    came_from[neighbor] = current
    
    return None

# A* Algorithm
def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()
    
    while open_set:
        _, cost, current = heapq.heappop(open_set)
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        visited.add(current)
        
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0] + dr, current[1] + dc)
            if valid(*neighbor):
                tentative_g_score = cost + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
    
    return None

# Path reconstruction
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Visualisasi Map
def visualize_path(path, title="Path"):
    temp_map = [row.copy() for row in city_map]
    for r, c in path:
        if temp_map[r][c] == '.':
            temp_map[r][c] = '*'
    
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title(title)
    for r in range(rows):
        for c in range(cols):
            if temp_map[r][c] == 'S':
                color = 'green'
            elif temp_map[r][c] == 'H':
                color = 'red'
            elif temp_map[r][c] == 'T':
                color = 'black'
            elif temp_map[r][c] == '*':
                color = 'blue'
            else:
                color = 'white'
            ax.add_patch(plt.Rectangle((c, rows-1-r), 1, 1, color=color, ec='gray'))
    
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.grid(True)
    plt.gca().invert_yaxis()
    plt.show()

# Tes GBFS
gbfs_path = gbfs(start, goal)
if gbfs_path:
    print("GBFS path:", gbfs_path)
    visualize_path(gbfs_path, title="GBFS Path")

# Tes A*
astar_path = astar(start, goal)
if astar_path:
    print("A* path:", astar_path)
    visualize_path(astar_path, title="A* Path")