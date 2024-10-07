from flask import Flask, render_template, request, jsonify
import heapq
from collections import deque

app = Flask(__name__)

# Dijkstra's algorithm
def dijkstra(graph, start, end):
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {start: float('inf')}
    distances[start] = 0
    previous_nodes = {start: None}
    visited = set()

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node in visited:
            continue
        visited.add(current_node)

        if current_node == end:
            break

        for neighbor in graph.get(current_node, {}).keys():
            distance = current_distance + 1
            if neighbor not in visited and (neighbor not in distances or distance < distances[neighbor]):
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    path = []
    while end is not None:
        path.append(end)
        end = previous_nodes[end]
    path.reverse()
    return path

# A* algorithm
def a_star(graph, start, end):
    open_set = {start}
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, end)

    while open_set:
        current = min(open_set, key=lambda node: f_score[node])

        if current == end:
            return reconstruct_path(came_from, current)

        open_set.remove(current)

        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                if neighbor not in open_set:
                    open_set.add(neighbor)

    return []

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

# Breadth-First Search
def bfs(graph, start, end):
    queue = deque([start])
    visited = {start}
    came_from = {start: None}

    while queue:
        current = queue.popleft()

        if current == end:
            return reconstruct_path(came_from, current)

        for neighbor in graph.get(current, {}).keys():
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
                came_from[neighbor] = current

    return []

def reconstruct_path(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from.get(current)
    path.reverse()
    return path

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/path', methods=['POST'])
def path():
    data = request.json
    grid = data['grid']
    start = tuple(data['start'])
    end = tuple(data['end'])
    algorithm = data['algorithm']
    
    graph = {}
    for y in range(len(grid)):
        for x in range(len(grid[y])):
            if grid[y][x] == 1:  # obstacle
                continue
            neighbors = []
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid) and grid[ny][nx] != 1:
                    neighbors.append((nx, ny))
            graph[(x, y)] = {n: 1 for n in neighbors}

    if algorithm == 'dijkstra':
        path = dijkstra(graph, start, end)
    elif algorithm == 'a_star':
        path = a_star(graph, start, end)
    elif algorithm == 'bfs':
        path = bfs(graph, start, end)
    else:
        path = []

    return jsonify(path)

if __name__ == '__main__':
    app.run(debug=True)
