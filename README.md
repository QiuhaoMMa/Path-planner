# Path-planner

This repository provides the official MATLAB implementation and datasets used in the paper **"A Standard Framework for Testing and Benchmarking of 2D RRT Path Planner"**. It offers a unified simulation environment for systematically testing and comparing multiple sampling-based path-planning algorithms across diverse 2D scenarios. The framework is intended for benchmarking, analysis, and educational use in robotics and motion planning research.

---

## 📌 Features

- **Implemented Algorithms**
  - RRT (Rapidly-exploring Random Tree)
  - RRT* (Optimal RRT with rewiring)
  - Informed RRT* (Goal-biased sampling)
  - RRT*-Smart (RRT* with local path refinement)
  - LazyRRT (Lazy collision checking)

- **Benchmark Scenarios**
  - Macro Obstacles
  - Micro Obstacles
  - Multi-Obstacle Indoor
  - Indoor Room
  - Circular Maze
  - Serpentine Corridor
  - Narrow Path Corridor
  - Narrow Gate Passage
  - Branching Pathways
  - Enclosed Starting
  - Enclosed Goal

Each scenario has three levels of difficulty: Easy, Modest, and Difficult, to test algorithm performance under increasing complexity.

---

## ⚙️ Requirements

- MATLAB R2022a or newer
- Required Toolboxes:
  - Robotics System Toolbox
  - Statistics and Machine Learning Toolbox
  - (Optional) Parallel Computing Toolbox for faster batch tests

---

## 🚀 Quick Start

1. **Clone the repository**
   ```bash
   git clone https://github.com/QiuhaoMMa/Path-planner.git
   cd Path-planner

2. **Open MATLAB**
   set the working folder to the repo root.

3. **Run script**
   Run the main_benchmark.m script


## 📊 Example Results

The simulation outputs include:

Comparison plots of generated paths for all planners on the same map.

Example visualizations for Easy, Modest, and Difficult levels.

Performance data such as:

Average path length

Average computation time

Standard deviation for repeat runs

