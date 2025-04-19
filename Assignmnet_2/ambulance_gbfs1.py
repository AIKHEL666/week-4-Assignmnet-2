import heapq

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

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Jalankan
path = gbfs(start, goal)

# Print hasil
if path:
    for r, c in path:
        if city_map[r][c] == '.':
            city_map[r][c] = '*'
    for row in city_map:
        print(' '.join(row))
else:
    print("No path found.")