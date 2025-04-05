import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import time
from matplotlib.table import Table
import sys


class PathAnimationMazeSolver:
    def __init__(self, dim=20, p=0.2):
        self.dim = dim
        self.p = p
        self.grid = self.generate_valid_maze()
        self.graph = self.create_maze_graph()
        self.start = (0, 0)
        self.goal = (dim - 1, dim - 1)

        self.fig = None
        self.axs = None
        self.current_markers = []
        self.start_markers = []
        self.goal_markers = []
        self.visited_nodes = []
        self.current_paths = []
        self.final_paths = []
        self.animation_running = []
        self.algorithm_names = []
        self.times = []
        self.animations = []
        self.path_lines = []
        self.table_ax = None

    def generate_valid_maze(self):
        """Generate maze ensuring start node has at least one neighbor"""
        while True:
            maze = np.random.choice([0, 1], size=(self.dim, self.dim), p=[1 - self.p, self.p])
            maze[0, 0] = 0
            maze[-1, -1] = 0

            # Check if start node has neighbors
            has_neighbors = False
            if self.dim > 1:
                if maze[1, 0] == 0 or maze[0, 1] == 0:
                    has_neighbors = True
            if has_neighbors:
                return maze

    def create_maze_graph(self):
        G = nx.Graph()
        for i in range(self.dim):
            for j in range(self.dim):
                if self.grid[i, j] == 0:
                    G.add_node((i, j))
                    if i > 0 and self.grid[i - 1, j] == 0:
                        G.add_edge((i, j), (i - 1, j))
                    if j > 0 and self.grid[i, j - 1] == 0:
                        G.add_edge((i, j), (i, j - 1))
        return G

    def draw_maze_background(self, ax):
        for i in range(self.dim):
            for j in range(self.dim):
                color = '#333333' if self.grid[i, j] == 1 else '#f0f0f0'
                ax.add_patch(Rectangle((j - 0.5, -i - 0.5), 1, 1,
                                   facecolor=color, edgecolor='#888888', lw=0.5))

    def heuristic(self, a, b, type='manhattan'):
        if type == 'manhattan':
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def dfs_generator(self, idx):
        start_time = time.time()
        stack = [(self.start, [self.start])]
        while stack and self.animation_running[idx]:
            vertex, path = stack.pop()
            if vertex not in self.visited_nodes[idx]:
                self.visited_nodes[idx].add(vertex)
                self.current_markers[idx].center = (vertex[1], -vertex[0])
                self.current_paths[idx] = path
                if vertex == self.goal:
                    self.final_paths[idx] = path
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    break
                for neighbor in sorted(self.graph[vertex], reverse=True):
                    stack.append((neighbor, path + [neighbor]))
                yield
                time.sleep(0.05)  # Added delay to slow down animation

    def bfs_generator(self, idx):
        start_time = time.time()
        queue = deque([(self.start, [self.start])])
        while queue and self.animation_running[idx]:
            vertex, path = queue.popleft()
            if vertex not in self.visited_nodes[idx]:
                self.visited_nodes[idx].add(vertex)
                self.current_markers[idx].center = (vertex[1], -vertex[0])
                self.current_paths[idx] = path
                if vertex == self.goal:
                    self.final_paths[idx] = path
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    break
                for neighbor in sorted(self.graph[vertex]):
                    queue.append((neighbor, path + [neighbor]))
                yield
                time.sleep(0.05)  # Added delay to slow down animation

    def a_star_generator(self, idx, heuristic_type='manhattan'):
        start_time = time.time()
        open_set = {self.start}
        came_from = {}
        g_score = {node: float('inf') for node in self.graph.nodes()}
        g_score[self.start] = 0
        f_score = {node: float('inf') for node in self.graph.nodes()}
        f_score[self.start] = self.heuristic(self.start, self.goal, heuristic_type)

        while open_set and self.animation_running[idx]:
            current = min(open_set, key=lambda node: f_score[node])
            self.visited_nodes[idx].add(current)
            self.current_markers[idx].center = (current[1], -current[0])

            path = []
            temp = current
            while temp in came_from:
                path.append(temp)
                temp = came_from[temp]
            path.append(self.start)
            self.current_paths[idx] = path[::-1]

            if current == self.goal:
                self.final_paths[idx] = self.current_paths[idx]
                self.animation_running[idx] = False
                self.times[idx] = time.time() - start_time
                yield
                break

            open_set.remove(current)
            for neighbor in self.graph[current]:
                tentative_g = g_score[current] + 1
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, self.goal, heuristic_type)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
            yield
            time.sleep(0.05)  # Added delay to slow down animation

    def bidirectional_bfs_generator(self, idx):
        start_time = time.time()
        if self.start == self.goal:
            self.final_paths[idx] = [self.start]
            self.animation_running[idx] = False
            self.times[idx] = time.time() - start_time
            yield
            return

        queue_f = deque([(self.start, [self.start])])
        queue_b = deque([(self.goal, [self.goal])])
        visited_f = {self.start: [self.start]}
        visited_b = {self.goal: [self.goal]}

        while queue_f and queue_b and self.animation_running[idx]:
            # Forward search
            current_f, path_f = queue_f.popleft()
            self.visited_nodes[idx].add(current_f)
            self.current_markers[idx].center = (current_f[1], -current_f[0])
            self.current_paths[idx] = path_f

            for neighbor in sorted(self.graph[current_f]):
                if neighbor in visited_b:
                    self.final_paths[idx] = path_f + visited_b[neighbor][::-1]
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    return
                if neighbor not in visited_f:
                    visited_f[neighbor] = path_f + [neighbor]
                    queue_f.append((neighbor, visited_f[neighbor]))
            yield
            time.sleep(0.05)  # Added delay to slow down animation

            # Backward search
            current_b, path_b = queue_b.popleft()
            self.visited_nodes[idx].add(current_b)
            self.current_markers[idx].center = (current_b[1], -current_b[0])
            self.current_paths[idx] = path_b

            for neighbor in sorted(self.graph[current_b]):
                if neighbor in visited_f:
                    self.final_paths[idx] = visited_f[neighbor] + path_b[::-1]
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    return
                if neighbor not in visited_b:
                    visited_b[neighbor] = path_b + [neighbor]
                    queue_b.append((neighbor, visited_b[neighbor]))
            yield
            time.sleep(0.05)  # Added delay to slow down animation

    def update_animation(self, frame, idx):
        ax = self.axs[idx]

        # Clear previous visualization elements except maze and markers
        while len(ax.patches) > 2 + self.dim * self.dim:
            ax.patches[-1].remove()

        # Clear previous path lines by removing the artist from the figure
        if len(self.path_lines) > idx and self.path_lines[idx] is not None:
            if self.path_lines[idx] in ax.lines:
                self.path_lines[idx].remove()
            self.path_lines[idx] = None

        # Draw visited nodes
        visited_list = list(self.visited_nodes[idx])
        for i, node in enumerate(visited_list):
            alpha = 0.3 + 0.7 * (i / len(visited_list))
            ax.add_patch(Rectangle((node[1] - 0.5, -node[0] - 0.5), 1, 1,
                                   facecolor='#88ccff', alpha=alpha * 0.5, edgecolor='none'))

        # Draw current path as a line
        if len(self.current_paths) > idx and self.current_paths[idx]:
            path_x = [node[1] for node in self.current_paths[idx]]
            path_y = [-node[0] for node in self.current_paths[idx]]
            self.path_lines[idx], = ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.6, zorder=4)

        # Draw final path if found
        if len(self.final_paths) > idx and self.final_paths[idx]:
            final_path_x = [node[1] for node in self.final_paths[idx]]
            final_path_y = [-node[0] for node in self.final_paths[idx]]
            ax.plot(final_path_x, final_path_y, 'r-', linewidth=3, alpha=0.8, zorder=5)

        # Update title
        time_str = f"{self.times[idx]:.3f}s" if self.times[idx] else "Running..."
        path_len = len(self.final_paths[idx]) - 1 if len(self.final_paths) > idx and self.final_paths[
            idx] else 'Finding...'
        ax.set_title(f"{self.algorithm_names[idx]}\n"
                     f"Explored: {len(self.visited_nodes[idx])} | "
                     f"Path: {path_len}\n"
                     f"Time: {time_str}")

        return []

    def show_final_results(self):
        self.table_ax.clear()
        self.table_ax.axis('off')

        data = [
            ["Metric", *self.algorithm_names],
            ["Time Taken (s)", *[f"{time:.3f}" for time in self.times]],
            ["Nodes Explored", *[str(len(visited)) for visited in self.visited_nodes]],
            ["Path Length", *[str(len(path) - 1 if path else 'N/A') for path in self.final_paths]]
        ]

        table = Table(self.table_ax, bbox=[0.1, 0.1, 0.8, 0.8])
        for i, row in enumerate(data):
            for j, cell in enumerate(row):
                table.add_cell(i, j, width=0.3, height=0.15, text=cell,
                               loc='center', facecolor='lightgray' if i == 0 else 'white')
        table.auto_set_font_size(False)
        table.set_fontsize(12)
        self.table_ax.add_table(table)

        plt.draw()
        plt.pause(3)  # Show results for 3 seconds
        plt.close()  # Then close the window
        sys.exit(0)  # Exit the program

    def solve_with_animation_pair(self, algorithms):
        # Create figure with adjusted layout (10% larger than original)
        self.fig = plt.figure(figsize=(16.5, 11))  # Original was (15, 10)

        # Use GridSpec for layout management
        gs = plt.GridSpec(2, 2, height_ratios=[3, 1], hspace=0.4)

        # Create axes for maze visualizations (10% larger)
        self.axs = [self.fig.add_subplot(gs[0, 0]), self.fig.add_subplot(gs[0, 1])]

        # Create axis for comparison table
        self.table_ax = self.fig.add_subplot(gs[1, :])
        self.table_ax.axis('off')

        # Initialize path lines
        self.path_lines = [None] * len(algorithms)

        for ax in self.axs:
            # Adjust limits to account for larger size
            ax.set_xlim(-0.55, self.dim - 0.45)
            ax.set_ylim(-self.dim + 0.45, 0.55)
            ax.set_aspect('equal')
            ax.axis('off')
            self.draw_maze_background(ax)

        # Create markers (slightly larger)
        self.start_markers = [Circle((self.start[1], -self.start[0]), 0.33,  # Original was 0.3
                                     color='lime', zorder=10) for _ in range(len(algorithms))]
        self.goal_markers = [Circle((self.goal[1], -self.goal[0]), 0.33,  # Original was 0.3
                                    color='gold', zorder=10) for _ in range(len(algorithms))]
        self.current_markers = [Circle((0, 0), 0.22, color='blue', zorder=10)  # Original was 0.2
                                for _ in range(len(algorithms))]

        for i in range(len(algorithms)):
            self.axs[i].add_patch(self.start_markers[i])
            self.axs[i].add_patch(self.goal_markers[i])
            self.axs[i].add_patch(self.current_markers[i])

        self.visited_nodes = [set() for _ in range(len(algorithms))]
        self.current_paths = [[] for _ in range(len(algorithms))]
        self.final_paths = [[] for _ in range(len(algorithms))]
        self.animation_running = [True] * len(algorithms)
        self.algorithm_names = [algo_map[algo] for algo in algorithms]
        self.times = [0] * len(algorithms)

        generators = []
        for i, algo in enumerate(algorithms):
            if algo == 'dfs':
                generators.append(self.dfs_generator(i))
            elif algo == 'bfs':
                generators.append(self.bfs_generator(i))
            elif algo == 'astar_manhattan':
                generators.append(self.a_star_generator(i, 'manhattan'))
            elif algo == 'astar_euclidean':
                generators.append(self.a_star_generator(i, 'euclidean'))
            elif algo == 'bidirectional_bfs':
                generators.append(self.bidirectional_bfs_generator(i))

        # Slower animation speed (200ms interval instead of 100ms)
        self.animations = [
            FuncAnimation(self.fig, lambda frame, idx=i: self.update_animation(frame, idx),
                          frames=generators[i], interval=200, blit=False, repeat=False, save_count=1000)
            for i in range(len(algorithms))
        ]

        def update_table():
            self.table_ax.clear()
            self.table_ax.axis('off')

            if not any(self.animation_running):
                data = [
                    ["Metric", *self.algorithm_names],
                    ["Time Taken (s)", *[f"{time:.3f}" for time in self.times]],
                    ["Nodes Explored", *[str(len(visited)) for visited in self.visited_nodes]],
                    ["Path Length", *[str(len(path) - 1 if path else 'N/A') for path in self.final_paths]]
                ]

                table = Table(self.table_ax, bbox=[0.1, 0.1, 0.8, 0.8])
                for i, row in enumerate(data):
                    for j, cell in enumerate(row):
                        table.add_cell(i, j, width=0.3, height=0.15, text=cell,
                                       loc='center', facecolor='lightgray' if i == 0 else 'white')
                table.auto_set_font_size(False)
                table.set_fontsize(12)
                self.table_ax.add_table(table)
                plt.draw()
                plt.pause(30)  # Show results for 30 seconds
                plt.close()  # Then close the window
                sys.exit(0)  # Exit the program

        def check_animation_completion(frame):
            update_table()
            if not any(self.animation_running):
                return False
            return True

        self.animations.append(FuncAnimation(self.fig, check_animation_completion,
                                             frames=range(10000), interval=200, repeat=False))

        plt.show()


def main():
    print("Maze Runner ")
    print("-----------")
    dim = 20
    p = 0.2
    solver = PathAnimationMazeSolver(dim, p)

    print("\nAvailable Comparisons:")
    print("1. DFS vs BFS")
    print("2. A* Manhattan vs A* Euclidean")
    print("3. BFS vs Bi-Directional BFS")
    choice = input("\nSelect comparison (1-3): ").strip()

    global algo_map
    algo_map = {
        'dfs': "Depth-First Search",
        'bfs': "Breadth-First Search",
        'astar_manhattan': "A* (Manhattan Heuristic)",
        'astar_euclidean': "A* (Euclidean Heuristic)",
        'bidirectional_bfs': "Bi-Directional BFS"
    }

    comparisons = {
        '1': ['dfs', 'bfs'],
        '2': ['astar_manhattan', 'astar_euclidean'],
        '3': ['bfs', 'bidirectional_bfs']
    }

    if choice in comparisons:
        solver.solve_with_animation_pair(comparisons[choice])
    else:
        print("Invalid selection. Please choose 1-3.")


if __name__ == "__main__":
    main()
