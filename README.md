
 Maze Pathfinding Algorithm Visualizer

Overview

The **Maze Pathfinding Algorithm Visualizer** is a Python-based tool designed to visually demonstrate and compare various pathfinding algorithms (DFS, BFS, A*, Bi-Directional BFS). This tool allows users to experiment with different algorithms and view their search processes in real-time, providing a side-by-side comparison of their performance. The visualizer helps users understand the mechanics of each algorithm, including their efficiency, search patterns, and trade-offs in terms of time and space complexity.

### Key Features:
- **Side-by-Side Algorithm Comparison**: Visualizes two algorithms solving the same maze simultaneously.
- **Real-Time Animation**: Animates the search process, showing the exploration of nodes and the final path.
- **Performance Metrics**: Displays the time taken, nodes explored, and path length for each algorithm.
- **Customizable Maze Settings**: Allows users to adjust maze size and obstacle density.
- **Clear Path Visualization**: Uses distinct colors to represent explored nodes, the current path, and the final solution.

---

## **Algorithms Implemented**

1. **Depth-First Search (DFS)**: Explores a maze by going as deep as possible along each branch before backtracking.
2. **Breadth-First Search (BFS)**: Explores the maze level by level, guaranteeing the shortest path in an unweighted grid.
3. **A* Search**: An informed search algorithm that uses heuristics to find the shortest path more efficiently.
   - **Manhattan Distance**: Suitable for grids with only horizontal and vertical movements.
   - **Euclidean Distance**: Suitable for more general environments with diagonal movement.
4. **Bi-Directional BFS**: Runs two BFS searches simultaneously—one from the start and one from the goal, meeting in the middle.

---

## **Installation**

To run the **Maze Pathfinding Algorithm Visualizer** on your local machine, follow these steps:

### **1. Clone the Repository**

Clone this repository to your local machine using the following command:

```bash
[git clone https://github.com/Kitty-Kat1997/Maze-Solver.git]
```

### **2. Install Required Dependencies**

You will need **Python 3.x** installed on your system along with the following libraries:
- **matplotlib**: For visualizing the maze and animating the algorithms.
- **networkx**: For creating and manipulating the graph representation of the maze.
- **numpy**: For handling arrays and matrix manipulations.

To install the required dependencies, navigate to the project folder and run:

```bash
pip install -r requirements.txt
```

### **3. Create the `requirements.txt` file**

If you don't have the `requirements.txt` file, create one with the following content:

```
matplotlib
networkx
numpy
```

---

## **Usage**

### **Running the Program**

After installing the dependencies, you can run the program by executing the following command:

```bash
python main.py
```

The program will display a graphical user interface where you can:
- Select the maze size and obstacle density.
- Choose which algorithms you would like to compare.
- Watch the algorithms run in real-time, visualizing their search process and the final path.

### **Algorithm Comparison**

You can compare two algorithms side-by-side on the same maze. The available algorithm pairs are:
1. **DFS vs BFS**
2. **A* (Manhattan) vs A* (Euclidean)**
3. **BFS vs Bi-Directional BFS**

### **Adjusting Maze Settings**

Before starting the algorithms, you can adjust the maze settings:
- **Maze Size**: Choose the dimensions of the maze (e.g., 20x20).
- **Obstacle Density**: Set the probability that a cell will be an obstacle (e.g., 0.2).

The program will generate a random maze based on these settings, ensuring that the start and goal positions are always open.

---

## **Performance Metrics**

The **Maze Pathfinding Algorithm Visualizer** provides real-time performance metrics for each algorithm, including:
- **Time Taken**: The total time taken by the algorithm to find the solution (in seconds).
- **Nodes Explored**: The number of nodes the algorithm visited during its search process.
- **Path Length**: The length of the final path from the start to the goal.

These metrics are displayed in a table below the visualized maze, making it easy to compare the efficiency of different algorithms.

---

## **Contributing**

If you’d like to contribute to this project, please feel free to fork the repository and submit a pull request. Contributions are welcome for:
- Bug fixes
- New features (such as additional pathfinding algorithms)
- Performance improvements

Please follow standard coding conventions and ensure that your code is well-documented.

---


## **References**

- **Introduction to Algorithms** (3rd ed.) by Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, and Clifford Stein.
- **Artificial Intelligence: A Modern Approach** (3rd ed.) by Stuart Russell and Peter Norvig.
- **NetworkX Documentation**: https://networkx.github.io/
- **Matplotlib Documentation**: https://matplotlib.org/stable/contents.html
- **Wikipedia** pages on DFS, BFS, A*, and Bi-Directional BFS.

---

## **Troubleshooting**

If you encounter issues, consider the following steps:
- **Dependencies**: Ensure that all required dependencies are installed using `pip install -r requirements.txt`.
- **Visualization Issues**: If the maze is not displaying correctly, try updating your **matplotlib** and **numpy** libraries.

For additional support, feel free to open an issue on the GitHub repository.

---

This **README** provides all the necessary details for users to install, use, and contribute to the **Maze Pathfinding Algorithm Visualizer** project. Let me know if you need any further adjustments!
