# Dynamic Pathfinding Agent

A grid-based pathfinding visualizer built in Python using Tkinter. Implements **Greedy Best-First Search (GBFS)** and **A\*** with real-time dynamic obstacle re-planning.

---

## Features

- Configurable grid size (rows x columns)
- Random map generation with adjustable obstacle density
- Interactive map editor (click/drag to place or remove walls)
- Toggle between GBFS and A* algorithms
- Three heuristics: Manhattan, Euclidean, Chebyshev
- 4-directional and 8-directional (diagonal) movement
- Dynamic obstacle spawning with automatic real-time re-planning
- Animated visualisation of frontier, visited nodes, and final path
- Live metrics: Nodes Visited, Path Cost, Execution Time

---

## Requirements

- Python 3.8 or higher
- Tkinter (usually included with Python)

> No external packages required. Tkinter comes bundled with most Python installations.

---

## Installation

### Step 1 — Install Python

Download from [https://www.python.org/downloads/](https://www.python.org/downloads/)

Make sure to check **"Add Python to PATH"** during installation.

### Step 2 — Verify Tkinter is available

```bash
python -m tkinter
```

A small test window should appear. If it does not:

**Ubuntu / Debian:**
```bash
sudo apt-get install python3-tk
```

**Fedora:**
```bash
sudo dnf install python3-tkinter
```

**Windows / macOS:** Tkinter is included by default with the official Python installer.

---

## How to Run

```bash
python pathfinder.py
```

Or on some systems:

```bash
python3 pathfinder.py
```

---

## Usage Guide

| Control | Description |
|---|---|
| **Rows / Cols spinbox** | Set grid dimensions |
| **Obstacle Density slider** | Set wall coverage percentage |
| **Generate Map** | Create a random maze |
| **Clear Map** | Remove all walls |
| **Place / Remove Wall** | Click or drag on grid to toggle walls |
| **Move Start Node** | Click a cell to move the start position |
| **Move Goal Node** | Click a cell to move the goal position |
| **Algorithm radio** | Choose GBFS or A* |
| **Heuristic radio** | Choose Manhattan, Euclidean, or Chebyshev |
| **Dynamic Obstacles** | Enable random wall spawning during movement |
| **8-Directional Movement** | Allow diagonal movement |
| **Find Path** | Run the selected algorithm and animate |
| **Reset Visualisation** | Clear colours, keep walls |
| **Stop** | Cancel current animation |

---

## Colour Legend

| Colour | Meaning |
|---|---|
| 🟢 Green | Start node |
| 🔴 Red | Goal node |
| ⬛ Dark | Empty cell |
| 🔵 Blue | Visited / expanded nodes |
| 🟡 Yellow | Frontier nodes (in priority queue) |
| 🟩 Teal | Final path |
| 🟠 Orange | Agent current position |
| 🔴 Bright Red | Newly spawned dynamic wall |

---

## Algorithm Details

### Greedy Best-First Search (GBFS)
- Expansion order: lowest `h(n)` (heuristic only)
- Fast but **not optimal**
- Good for: quick navigation, dynamic environments

### A* Search
- Expansion order: lowest `f(n) = g(n) + h(n)`
- **Optimal** with admissible heuristic
- Good for: shortest path guarantee

---

## Project Structure

```
dynamic-pathfinding-agent/
├── pathfinder.py     # Main application
└── README.md         # This file
```

---

## Dependencies

```
No pip installs required.
Python standard library only: tkinter, heapq, random, math, time, collections
```

---

## License

This project was developed as part of an Artificial Intelligence course assignment.
