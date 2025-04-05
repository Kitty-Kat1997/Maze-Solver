"""
Maze Pathfinding Algorithm Visualizer

This program compares different pathfinding algorithms (DFS, BFS, A*, Bi-Directional BFS)
by animating their search process through randomly generated mazes. The visualization shows:
- The maze grid with obstacles
- The algorithm's exploration process
- The final path found
- Performance metrics comparison

Key Features:
- Side-by-side algorithm comparison
- Adjustable maze size and obstacle density
- Clear visualization of search patterns
- Automatic performance comparison
"""

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
    """
    Main class for maze pathfinding visualization and comparison

    Attributes:
        dim: Dimension of the square maze (default 20x20)
        p: Probability of a cell being an obstacle (default 0.2)
        grid: 2D numpy array representing the maze (0=path, 1=obstacle)
        graph: NetworkX graph representation of the maze
        start: Starting position (always top-left corner)
        goal: Goal position (always bottom-right corner)
    """

    def __init__(self, dim=20, p=0.2):
        """Initialize maze solver with given dimensions and obstacle probability"""
        self.dim = dim
        self.p = p
        self.grid = self.generate_valid_maze()  # Generate valid maze
        self.graph = self.create_maze_graph()  # Convert maze to graph
        self.start = (0, 0)  # Start position (top-left)
        self.goal = (dim - 1, dim - 1)  # Goal position (bottom-right)

        # Visualization components
        self.fig = None  # Main figure
        self.axs = None  # Axes for maze displays
        self.current_markers = []  # Current position markers
        self.start_markers = []  # Start position markers
        self.goal_markers = []  # Goal position markers
        self.visited_nodes = []  # Sets of visited nodes for each algorithm
        self.current_paths = []  # Current paths being explored
        self.final_paths = []  # Final paths found
        self.animation_running = []  # Animation control flags
        self.algorithm_names = []  # Names of algorithms being compared
        self.times = []  # Execution times
        self.animations = []  # Animation objects
        self.path_lines = []  # Path line objects
        self.table_ax = None  # Axis for comparison table

    def generate_valid_maze(self):
        """
        Generate a valid maze where:
        - Start and goal positions are always open
        - Start has at least one neighbor
        - Uses random obstacle placement with probability p

        Returns:
            2D numpy array representing the maze (0=path, 1=obstacle)
        """
        while True:
            # Generate random maze with obstacle probability p
            maze = np.random.choice([0, 1], size=(self.dim, self.dim), p=[1 - self.p, self.p])
            # Ensure start and goal are always open
            maze[0, 0] = 0
            maze[-1, -1] = 0

            # Check if start node has at least one neighbor
            has_neighbors = False
            if self.dim > 1:
                if maze[1, 0] == 0 or maze[0, 1] == 0:
                    has_neighbors = True
            if has_neighbors:
                return maze

    def create_maze_graph(self):
        """
        Convert the maze grid into a graph representation where:
        - Each open cell is a node
        - Adjacent open cells are connected with edges

        Returns:
            NetworkX graph representing the maze
        """
        G = nx.Graph()
        for i in range(self.dim):
            for j in range(self.dim):
                if self.grid[i, j] == 0:  # If cell is open
                    G.add_node((i, j))
                    # Connect to open neighbors (left and above)
                    if i > 0 and self.grid[i - 1, j] == 0:
                        G.add_edge((i, j), (i - 1, j))
                    if j > 0 and self.grid[i, j - 1] == 0:
                        G.add_edge((i, j), (i, j - 1))
        return G

    def draw_maze_background(self, ax):
        """
        Draw the maze background on a given axis
        - Open cells: light gray
        - Obstacles: dark gray
        - Grid lines: gray

        Args:
            ax: Matplotlib axis to draw on
        """
        for i in range(self.dim):
            for j in range(self.dim):
                color = '#333333' if self.grid[i, j] == 1 else '#f0f0f0'
                ax.add_patch(Rectangle((j - 0.5, -i - 0.5), 1, 1,
                                       facecolor=color, edgecolor='#888888', lw=0.5))

    def heuristic(self, a, b, type='manhattan'):
        """
        Calculate heuristic distance between two points

        Args:
            a: First position (tuple)
            b: Second position (tuple)
            type: Type of heuristic ('manhattan' or 'euclidean')

        Returns:
            Heuristic distance between a and b
        """
        if type == 'manhattan':
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    # Pathfinding algorithm generators (each yields frames for animation)

    def dfs_generator(self, idx):
        """Depth-First Search algorithm generator"""
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
                time.sleep(0.05)  # Slow down animation

    def bfs_generator(self, idx):
        """Breadth-First Search algorithm generator"""
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
                time.sleep(0.05)  # Slow down animation

    def a_star_generator(self, idx, heuristic_type='manhattan'):
        """A* Search algorithm generator"""
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

            # Reconstruct path
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
            time.sleep(0.05)  # Slow down animation

    def bidirectional_bfs_generator(self, idx):
        """Bidirectional BFS algorithm generator"""
        start_time = time.time()
        if self.start == self.goal:
            self.final_paths[idx] = [self.start]
            self.animation_running[idx] = False
            self.times[idx] = time.time() - start_time
            yield
            return

        # Initialize forward and backward queues
        queue_f = deque([(self.start, [self.start])])
        queue_b = deque([(self.goal, [self.goal])])
        visited_f = {self.start: [self.start]}
        visited_b = {self.goal: [self.goal]}

        while queue_f and queue_b and self.animation_running[idx]:
            # Forward search step
            current_f, path_f = queue_f.popleft()
            self.visited_nodes[idx].add(current_f)
            self.current_markers[idx].center = (current_f[1], -current_f[0])
            self.current_paths[idx] = path_f

            for neighbor in sorted(self.graph[current_f]):
                if neighbor in visited_b:  # Path found!
                    self.final_paths[idx] = path_f + visited_b[neighbor][::-1]
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    return
                if neighbor not in visited_f:
                    visited_f[neighbor] = path_f + [neighbor]
                    queue_f.append((neighbor, visited_f[neighbor]))
            yield
            time.sleep(0.05)  # Slow down animation

            # Backward search step
            current_b, path_b = queue_b.popleft()
            self.visited_nodes[idx].add(current_b)
            self.current_markers[idx].center = (current_b[1], -current_b[0])
            self.current_paths[idx] = path_b

            for neighbor in sorted(self.graph[current_b]):
                if neighbor in visited_f:  # Path found!
                    self.final_paths[idx] = visited_f[neighbor] + path_b[::-1]
                    self.animation_running[idx] = False
                    self.times[idx] = time.time() - start_time
                    yield
                    return
                if neighbor not in visited_b:
                    visited_b[neighbor] = path_b + [neighbor]
                    queue_b.append((neighbor, visited_b[neighbor]))
            yield
            time.sleep(0.05)  # Slow down animation

    def update_animation(self, frame, idx):
        """
        Update function for animation - called for each frame

        Args:
            frame: Current frame data (from generator)
            idx: Index of algorithm being animated

        Returns:
            List of artists to be updated
        """
        ax = self.axs[idx]

        # Clear previous visualization elements except maze and markers
        while len(ax.patches) > 2 + self.dim * self.dim:
            ax.patches[-1].remove()

        # Clear previous path lines
        if len(self.path_lines) > idx and self.path_lines[idx] is not None:
            if self.path_lines[idx] in ax.lines:
                self.path_lines[idx].remove()
            self.path_lines[idx] = None

        # Draw visited nodes with fading blue color
        visited_list = list(self.visited_nodes[idx])
        for i, node in enumerate(visited_list):
            alpha = 0.3 + 0.7 * (i / len(visited_list))
            ax.add_patch(Rectangle((node[1] - 0.5, -node[0] - 0.5), 1, 1,
                                   facecolor='#88ccff', alpha=alpha * 0.5, edgecolor='none'))

        # Draw current path in blue
        if len(self.current_paths) > idx and self.current_paths[idx]:
            path_x = [node[1] for node in self.current_paths[idx]]
            path_y = [-node[0] for node in self.current_paths[idx]]
            self.path_lines[idx], = ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.6, zorder=4)

        # Draw final path in red if found
        if len(self.final_paths) > idx and self.final_paths[idx]:
            final_path_x = [node[1] for node in self.final_paths[idx]]
            final_path_y = [-node[0] for node in self.final_paths[idx]]
            ax.plot(final_path_x, final_path_y, 'r-', linewidth=3, alpha=0.8, zorder=5)

        # Update title with current stats
        time_str = f"{self.times[idx]:.3f}s" if self.times[idx] else "Running..."
        path_len = len(self.final_paths[idx]) - 1 if len(self.final_paths) > idx and self.final_paths[
            idx] else 'Finding...'
        ax.set_title(f"{self.algorithm_names[idx]}\n"
                     f"Explored: {len(self.visited_nodes[idx])} | "
                     f"Path: {path_len}\n"
                     f"Time: {time_str}")

        return []

    def show_final_results(self):
        """Display the final comparison table of results"""
        self.table_ax.clear()
        self.table_ax.axis('off')

        # Prepare comparison data
        data = [
            ["Metric", *self.algorithm_names],
            ["Time Taken (s)", *[f"{time:.3f}" for time in self.times]],
            ["Nodes Explored", *[str(len(visited)) for visited in self.visited_nodes]],
            ["Path Length", *[str(len(path) - 1 if path else 'N/A') for path in self.final_paths]]
        ]

        # Create and format comparison table
        table = Table(self.table_ax, bbox=[0.1, 0.1, 0.8, 0.8])
        for i, row in enumerate(data):
            for j, cell in enumerate(row):
                table.add_cell(i, j, width=0.3, height=0.15, text=cell,
                               loc='center', facecolor='lightgray' if i == 0 else 'white')
        table.auto_set_font_size(False)
        table.set_fontsize(12)
        self.table_ax.add_table(table)

        # Display results for 30 seconds before exiting
        plt.draw()
        plt.pause(30)
        plt.close()
        sys.exit(0)

    def solve_with_animation_pair(self, algorithms):
        """
        Main function to set up and run the comparison animation

        Args:
            algorithms: List of algorithm names to compare
        """
        # Set up figure with 10% larger size than original
        self.fig = plt.figure(figsize=(16.5, 11))  # Original was (15, 10)

        # Use GridSpec for layout management
        gs = plt.GridSpec(2, 2, height_ratios=[3, 1], hspace=0.4)

        # Create axes for maze visualizations (top row)
        self.axs = [self.fig.add_subplot(gs[0, 0]), self.fig.add_subplot(gs[0, 1])]

        # Create axis for comparison table (bottom row)
        self.table_ax = self.fig.add_subplot(gs[1, :])
        self.table_ax.axis('off')

        # Initialize path lines
        self.path_lines = [None] * len(algorithms)

        # Set up each maze display
        for ax in self.axs:
            # Adjust limits to account for larger size
            ax.set_xlim(-0.55, self.dim - 0.45)
            ax.set_ylim(-self.dim + 0.45, 0.55)
            ax.set_aspect('equal')
            ax.axis('off')
            self.draw_maze_background(ax)

        # Create and place markers (slightly larger than original)
        self.start_markers = [Circle((self.start[1], -self.start[0]), 0.33,  # Original was 0.3
                                     color='lime', zorder=10) for _ in range(len(algorithms))]
        self.goal_markers = [Circle((self.goal[1], -self.goal[0]), 0.33,  # Original was 0.3
                                    color='gold', zorder=10) for _ in range(len(algorithms))]
        self.current_markers = [Circle((0, 0), 0.22, color='blue', zorder=10)  # Original was 0.2
                                for _ in range(len(algorithms))]

        # Add markers to plots
        for i in range(len(algorithms)):
            self.axs[i].add_patch(self.start_markers[i])
            self.axs[i].add_patch(self.goal_markers[i])
            self.axs[i].add_patch(self.current_markers[i])

        # Initialize algorithm tracking variables
        self.visited_nodes = [set() for _ in range(len(algorithms))]
        self.current_paths = [[] for _ in range(len(algorithms))]
        self.final_paths = [[] for _ in range(len(algorithms))]
        self.animation_running = [True] * len(algorithms)
        self.algorithm_names = [algo_map[algo] for algo in algorithms]
        self.times = [0] * len(algorithms)

        # Set up algorithm generators
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

        # Create animations with slower speed (200ms interval)
        self.animations = [
            FuncAnimation(self.fig, lambda frame, idx=i: self.update_animation(frame, idx),
                          frames=generators[i], interval=200, blit=False, repeat=False, save_count=1000)
            for i in range(len(algorithms))
        ]

        def update_table():
            """Update the comparison table when algorithms complete"""
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
                plt.close()
                sys.exit(0)

        def check_animation_completion(frame):
            """Check if all animations have completed"""
            update_table()
            if not any(self.animation_running):
                return False
            return True

        # Add completion checker animation
        self.animations.append(FuncAnimation(self.fig, check_animation_completion,
                                             frames=range(10000), interval=200, repeat=False))

        plt.show()


def main():
    """Main function to run the maze solver comparison"""
    print("Maze Runner ")
    print("-----------")
    dim = 20
    p = 0.2
    solver = PathAnimationMazeSolver(dim, p)

    # Algorithm comparison options
    print("\nAvailable Comparisons:")
    print("1. DFS vs BFS")
    print("2. A* Manhattan vs A* Euclidean")
    print("3. BFS vs Bi-Directional BFS")
    choice = input("\nSelect comparison (1-3): ").strip()

    # Algorithm name mapping
    global algo_map
    algo_map = {
        'dfs': "Depth-First Search",
        'bfs': "Breadth-First Search",
        'astar_manhattan': "A* (Manhattan Heuristic)",
        'astar_euclidean': "A* (Euclidean Heuristic)",
        'bidirectional_bfs': "Bi-Directional BFS"
    }

    # Comparison configurations
    comparisons = {
        '1': ['dfs', 'bfs'],
        '2': ['astar_manhattan', 'astar_euclidean'],
        '3': ['bfs', 'bidirectional_bfs']
    }

    # Run selected comparison or show error
    if choice in comparisons:
        solver.solve_with_animation_pair(comparisons[choice])
    else:
        print("Invalid selection. Please choose 1-3.")


if __name__ == "__main__":
    main()
