import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import random
import math
import time
from collections import defaultdict

C = {
    "bg":        "#0D1117",
    "panel":     "#161B22",
    "border":    "#30363D",
    "start":     "#00D26A",
    "goal":      "#FF6B6B",
    "wall":      "#2D3748",
    "frontier":  "#F6C90E",
    "visited":   "#3D5A80",
    "path":      "#00BFA5",
    "agent":     "#FF9F1C",
    "empty":     "#1C2333",
    "text":      "#E6EDF3",
    "muted":     "#8B949E",
    "accent":    "#58A6FF",
    "new_wall":  "#FF4444",
}

CELL = 28
GAP  = 1
ANIM_MS  = 30
MOVE_MS  = 120
SPAWN_P  = 0.03

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def chebyshev(a, b):
    return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

HEURISTICS = {"Manhattan": manhattan, "Euclidean": euclidean, "Chebyshev": chebyshev}

DIRS_4 = [(-1,0),(1,0),(0,-1),(0,1)]
DIRS_8 = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def gbfs(grid, rows, cols, start, goal, h_fn, dirs):
    h = h_fn(start, goal)
    open_heap = [(h, 0, start)]
    came_from  = {start: None}
    visited    = []
    frontier_history = []
    counter    = 0

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        visited.append(cur)
        frontier_history.append(set(n for _,_,n in open_heap))

        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier_history

        for dr, dc in dirs:
            nb = (cur[0]+dr, cur[1]+dc)
            if 0 <= nb[0] < rows and 0 <= nb[1] < cols and grid[nb[0]][nb[1]] != 1 and nb not in came_from:
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap, (h_fn(nb, goal), counter, nb))

    return [], visited, frontier_history

def astar(grid, rows, cols, start, goal, h_fn, dirs):
    g_cost   = defaultdict(lambda: float('inf'))
    g_cost[start] = 0
    open_heap = [(h_fn(start, goal), 0, start)]
    came_from  = {start: None}
    closed     = set()
    visited    = []
    frontier_history = []
    counter    = 0

    while open_heap:
        f, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        visited.append(cur)
        frontier_history.append(set(n for _,_,n in open_heap) - closed)

        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier_history

        for dr, dc in dirs:
            nb = (cur[0]+dr, cur[1]+dc)
            if 0 <= nb[0] < rows and 0 <= nb[1] < cols and grid[nb[0]][nb[1]] != 1 and nb not in closed:
                step = math.hypot(dr, dc) if abs(dr)+abs(dc)==2 else 1.0
                tentative = g_cost[cur] + step
                if tentative < g_cost[nb]:
                    g_cost[nb] = tentative
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(open_heap, (tentative + h_fn(nb, goal), counter, nb))

    return [], visited, frontier_history

def _reconstruct(came_from, goal):
    path, cur = [], goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path

class PathfinderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent")
        self.root.configure(bg=C["bg"])
        self.root.resizable(True, True)

        self.rows = 20
        self.cols = 28
        self.grid = [[0]*self.cols for _ in range(self.rows)]
        self.start = (1, 1)
        self.goal  = (self.rows-2, self.cols-2)
        self.grid[self.start[0]][self.start[1]] = 0
        self.grid[self.goal[0]][self.goal[1]]   = 0

        self.mode = "wall"
        self.running = False
        self.dynamic_mode = tk.BooleanVar(value=False)
        self.diagonal_mode = tk.BooleanVar(value=False)
        self.algo_var   = tk.StringVar(value="A*")
        self.heur_var   = tk.StringVar(value="Manhattan")
        self.density_var = tk.DoubleVar(value=0.30)
        self.rows_var   = tk.IntVar(value=self.rows)
        self.cols_var   = tk.IntVar(value=self.cols)

        self.visited_cells  = set()
        self.frontier_cells = set()
        self.path_cells     = []
        self.agent_pos      = None
        self.agent_path     = []
        self.agent_idx      = 0
        self.anim_ids       = []

        self.stat_visited = tk.StringVar(value="0")
        self.stat_cost    = tk.StringVar(value="0")
        self.stat_time    = tk.StringVar(value="0 ms")
        self.stat_status  = tk.StringVar(value="Ready")

        self.draw_val = None

        self._build_ui()
        self._draw_grid()

    def _build_ui(self):
        panel = tk.Frame(self.root, bg=C["panel"], width=240)
        panel.pack(side=tk.LEFT, fill=tk.Y, padx=0, pady=0)
        panel.pack_propagate(False)

        self._section(panel, "⚙ GRID SETTINGS")
        self._spin_row(panel, "Rows", self.rows_var, 5, 40)
        self._spin_row(panel, "Cols", self.cols_var, 5, 60)
        self._slider_row(panel, "Obstacle Density", self.density_var, 0.05, 0.70)
        self._btn(panel, "Generate Map", self._generate_map, C["accent"])
        self._btn(panel, "Clear Map", self._clear_map, C["border"])

        self._section(panel, "✏ EDIT MODE")
        for label, val in [("🧱  Place / Remove Wall", "wall"),
                            ("🟢  Move Start Node", "start"),
                            ("🔴  Move Goal Node", "goal")]:
            b = tk.Button(panel, text=label, bg=C["border"], fg=C["text"],
                          activebackground=C["accent"], activeforeground=C["bg"],
                          relief=tk.FLAT, bd=0, pady=6, cursor="hand2",
                          command=lambda v=val: self._set_mode(v))
            b.pack(fill=tk.X, padx=12, pady=2)

        self._section(panel, "🔍 ALGORITHM")
        for a in ["A*", "GBFS"]:
            tk.Radiobutton(panel, text=a, variable=self.algo_var, value=a,
                           bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                           activebackground=C["panel"], activeforeground=C["accent"]).pack(anchor=tk.W, padx=20)

        self._section(panel, "📐 HEURISTIC")
        for h in HEURISTICS:
            tk.Radiobutton(panel, text=h, variable=self.heur_var, value=h,
                           bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                           activebackground=C["panel"], activeforeground=C["accent"]).pack(anchor=tk.W, padx=20)

        self._section(panel, "🎛 OPTIONS")
        tk.Checkbutton(panel, text="Dynamic Obstacles", variable=self.dynamic_mode,
                       bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                       activebackground=C["panel"], activeforeground=C["accent"]).pack(anchor=tk.W, padx=20)
        tk.Checkbutton(panel, text="8-Directional Movement", variable=self.diagonal_mode,
                       bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                       activebackground=C["panel"], activeforeground=C["accent"]).pack(anchor=tk.W, padx=20)

        self._section(panel, "🚀 CONTROL")
        self._btn(panel, "▶  Find Path", self._start_search, C["start"])
        self._btn(panel, "⟳  Reset Visualisation", self._reset_vis, "#6B48FF")
        self._btn(panel, "⏹  Stop", self._stop, C["goal"])

        self._section(panel, "📊 METRICS")
        for label, var, color in [
            ("Nodes Visited", self.stat_visited, C["visited"]),
            ("Path Cost",     self.stat_cost,    C["path"]),
            ("Time",          self.stat_time,    C["frontier"]),
        ]:
            row = tk.Frame(panel, bg=C["panel"])
            row.pack(fill=tk.X, padx=12, pady=2)
            tk.Label(row, text=label+":", bg=C["panel"], fg=C["muted"], font=("Consolas",9)).pack(side=tk.LEFT)
            tk.Label(row, textvariable=var, bg=C["panel"], fg=color, font=("Consolas",9,"bold")).pack(side=tk.RIGHT)

        tk.Label(panel, textvariable=self.stat_status, bg=C["panel"],
                 fg=C["accent"], font=("Consolas",9,"bold")).pack(pady=8)

        self._section(panel, "🎨 LEGEND")
        for label, color in [("Start", C["start"]), ("Goal", C["goal"]),
                              ("Wall", C["wall"]), ("Frontier", C["frontier"]),
                              ("Visited", C["visited"]), ("Path", C["path"]),
                              ("Agent", C["agent"]), ("New Wall", C["new_wall"])]:
            row = tk.Frame(panel, bg=C["panel"])
            row.pack(fill=tk.X, padx=12, pady=1)
            tk.Frame(row, bg=color, width=14, height=14).pack(side=tk.LEFT, padx=(0,6))
            tk.Label(row, text=label, bg=C["panel"], fg=C["text"], font=("Consolas",8)).pack(side=tk.LEFT)

        canvas_frame = tk.Frame(self.root, bg=C["bg"])
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)

        title = tk.Label(canvas_frame, text="DYNAMIC PATHFINDING AGENT",
                         bg=C["bg"], fg=C["accent"],
                         font=("Consolas", 14, "bold"))
        title.pack(pady=(4,2))

        sub = tk.Label(canvas_frame,
                       text="Click grid to edit • Use controls on the left to run",
                       bg=C["bg"], fg=C["muted"], font=("Consolas", 9))
        sub.pack(pady=(0,6))

        self.canvas_container = tk.Frame(canvas_frame, bg=C["bg"])
        self.canvas_container.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.canvas_container, bg=C["bg"],
                                highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)

    def _section(self, parent, title):
        tk.Label(parent, text=title, bg=C["panel"], fg=C["muted"],
                 font=("Consolas", 8, "bold")).pack(anchor=tk.W, padx=12, pady=(12,2))

    def _btn(self, parent, text, cmd, color):
        tk.Button(parent, text=text, command=cmd,
                  bg=color, fg=C["bg"] if color not in (C["border"], C["wall"]) else C["text"],
                  activebackground=C["text"], activeforeground=C["bg"],
                  relief=tk.FLAT, bd=0, pady=7, font=("Consolas",9,"bold"),
                  cursor="hand2").pack(fill=tk.X, padx=12, pady=3)

    def _spin_row(self, parent, label, var, mn, mx):
        row = tk.Frame(parent, bg=C["panel"])
        row.pack(fill=tk.X, padx=12, pady=2)
        tk.Label(row, text=label, bg=C["panel"], fg=C["text"],
                 font=("Consolas",9), width=6).pack(side=tk.LEFT)
        tk.Spinbox(row, from_=mn, to=mx, textvariable=var, width=5,
                   bg=C["bg"], fg=C["text"], insertbackground=C["text"],
                   relief=tk.FLAT).pack(side=tk.RIGHT)

    def _slider_row(self, parent, label, var, mn, mx):
        tk.Label(parent, text=label, bg=C["panel"], fg=C["text"],
                 font=("Consolas",9)).pack(anchor=tk.W, padx=12)
        tk.Scale(parent, variable=var, from_=mn, to=mx, resolution=0.05,
                 orient=tk.HORIZONTAL, bg=C["panel"], fg=C["text"],
                 troughcolor=C["bg"], highlightthickness=0,
                 showvalue=True).pack(fill=tk.X, padx=12)

    def _cell_coords(self, r, c):
        x0 = c*(CELL+GAP)
        y0 = r*(CELL+GAP)
        return x0, y0, x0+CELL, y0+CELL

    def _draw_grid(self):
        self.canvas.delete("all")
        self.rects = {}
        for r in range(self.rows):
            for c in range(self.cols):
                x0, y0, x1, y1 = self._cell_coords(r, c)
                color = self._cell_color(r, c)
                rect = self.canvas.create_rectangle(x0, y0, x1, y1,
                                                    fill=color, outline=C["bg"], width=GAP)
                self.rects[(r,c)] = rect

    def _cell_color(self, r, c):
        pos = (r, c)
        if pos == self.start:              return C["start"]
        if pos == self.goal:               return C["goal"]
        if pos == self.agent_pos:          return C["agent"]
        if self.grid[r][c] == 2:           return C["new_wall"]
        if self.grid[r][c] == 1:           return C["wall"]
        if pos in self.path_cells:         return C["path"]
        if pos in self.frontier_cells:     return C["frontier"]
        if pos in self.visited_cells:      return C["visited"]
        return C["empty"]

    def _update_cell(self, r, c):
        if (r,c) in self.rects:
            self.canvas.itemconfig(self.rects[(r,c)], fill=self._cell_color(r,c))

    def _refresh_all(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self._update_cell(r, c)

    def _set_mode(self, m):
        self.mode = m

    def _canvas_to_cell(self, x, y):
        c = x // (CELL+GAP)
        r = y // (CELL+GAP)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None, None

    def _on_click(self, event):
        if self.running: return
        r, c = self._canvas_to_cell(event.x, event.y)
        if r is None: return
        pos = (r, c)
        if self.mode == "wall" and pos not in (self.start, self.goal):
            self.draw_val = 0 if self.grid[r][c] else 1
            self.grid[r][c] = self.draw_val
            self._update_cell(r, c)
        else:
            self.draw_val = None
            self._apply_edit(r, c)

    def _on_drag(self, event):
        if self.running: return
        r, c = self._canvas_to_cell(event.x, event.y)
        if r is None: return
        pos = (r, c)
        if self.mode == "wall":
            if pos in (self.start, self.goal): return
            if self.draw_val is not None:
                if self.grid[r][c] != self.draw_val:
                    self.grid[r][c] = self.draw_val
                    self._update_cell(r, c)
        else:
            self._apply_edit(r, c)

    def _apply_edit(self, r, c):
        pos = (r, c)
        if self.mode == "wall":
            if pos in (self.start, self.goal): return
            self.grid[r][c] = 0 if self.grid[r][c] else 1
            self._update_cell(r, c)
        elif self.mode == "start":
            if pos == self.goal: return
            old = self.start
            self.start = pos
            self.grid[r][c] = 0
            self._update_cell(*old)
            self._update_cell(r, c)
        elif self.mode == "goal":
            if pos == self.start: return
            old = self.goal
            self.goal = pos
            self.grid[r][c] = 0
            self._update_cell(*old)
            self._update_cell(r, c)

    def _generate_map(self):
        if self.running: return
        self._stop()
        try:
            self.rows = int(self.rows_var.get())
            self.cols = int(self.cols_var.get())
        except:
            pass
        self.start = (min(self.start[0], self.rows-1), min(self.start[1], self.cols-1))
        self.goal  = (min(self.goal[0],  self.rows-1), min(self.goal[1],  self.cols-1))
        if self.start == self.goal:
            if self.goal[0] > 0:
                self.start = (self.goal[0]-1, self.goal[1])
            elif self.goal[0] < self.rows-1:
                self.start = (self.goal[0]+1, self.goal[1])
        
        self.grid = [[0]*self.cols for _ in range(self.rows)]
        density = self.density_var.get()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in (self.start, self.goal):
                    self.grid[r][c] = 1 if random.random() < density else 0
        self._reset_vis()
        self._draw_grid()

    def _clear_map(self):
        if self.running: return
        self._stop()
        try:
            self.rows = int(self.rows_var.get())
            self.cols = int(self.cols_var.get())
        except:
            pass
        self.start = (min(self.start[0], self.rows-1), min(self.start[1], self.cols-1))
        self.goal  = (min(self.goal[0],  self.rows-1), min(self.goal[1],  self.cols-1))
        if self.start == self.goal:
            if self.goal[0] > 0:
                self.start = (self.goal[0]-1, self.goal[1])
            elif self.goal[0] < self.rows-1:
                self.start = (self.goal[0]+1, self.goal[1])
        
        self.grid = [[0]*self.cols for _ in range(self.rows)]
        self._reset_vis()
        self._draw_grid()

    def _reset_vis(self):
        self._cancel_anims()
        self.visited_cells  = set()
        self.frontier_cells = set()
        self.path_cells     = set()
        self.agent_pos      = None
        self.agent_path     = []
        self.agent_idx      = 0
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 2:
                    self.grid[r][c] = 1
        self.stat_visited.set("0")
        self.stat_cost.set("0")
        self.stat_time.set("0 ms")
        self.stat_status.set("Ready")
        self._refresh_all()

    def _cancel_anims(self):
        for aid in self.anim_ids:
            self.root.after_cancel(aid)
        self.anim_ids = []
        self.running = False

    def _stop(self):
        self._cancel_anims()
        self.stat_status.set("Stopped")

    def _start_search(self):
        if self.running: return
        self._reset_vis()
        self._run_search(self.start)

    def _run_search(self, from_node):
        algo  = self.algo_var.get()
        h_fn  = HEURISTICS[self.heur_var.get()]
        dirs  = DIRS_8 if self.diagonal_mode.get() else DIRS_4

        t0 = time.perf_counter()
        if algo == "A*":
            path, visited, frontiers = astar(self.grid, self.rows, self.cols, from_node, self.goal, h_fn, dirs)
        else:
            path, visited, frontiers = gbfs(self.grid, self.rows, self.cols, from_node, self.goal, h_fn, dirs)
        elapsed = (time.perf_counter() - t0) * 1000

        self.stat_time.set(f"{elapsed:.1f} ms")
        self.stat_visited.set(str(len(visited)))

        if not path:
            self.stat_status.set("No path found!")
            messagebox.showwarning("No Path", "No path exists between Start and Goal.")
            return

        cost = 0
        for i in range(1, len(path)):
            dr = path[i][0]-path[i-1][0]
            dc = path[i][1]-path[i-1][1]
            cost += math.hypot(dr, dc) if abs(dr)+abs(dc)==2 else 1
        self.stat_cost.set(f"{cost:.1f}")
        self.stat_status.set(f"Animating ({algo})…")
        self.running = True

        self._animate_search(visited, frontiers, path)

    def _animate_search(self, visited, frontiers, path):
        step = [0]

        def tick():
            if not self.running: return
            if step[0] < len(visited):
                v = visited[step[0]]
                if v not in (self.start, self.goal):
                    self.visited_cells.add(v)
                    self._update_cell(*v)
                if step[0] < len(frontiers):
                    old_front = self.frontier_cells - {self.start, self.goal}
                    self.frontier_cells = frontiers[step[0]] - self.visited_cells - {self.start, self.goal}
                    for cell in old_front | self.frontier_cells:
                        self._update_cell(*cell)
                step[0] += 1
                aid = self.root.after(ANIM_MS, tick)
                self.anim_ids.append(aid)
            else:
                self.frontier_cells = set()
                self.path_cells = set(path)
                self._refresh_all()
                self.stat_status.set("Path found! Moving agent…")
                self.agent_path = path
                self.agent_idx  = 0
                aid = self.root.after(300, self._move_agent)
                self.anim_ids.append(aid)

        tick()

    def _move_agent(self):
        if not self.running: return
        if self.agent_idx >= len(self.agent_path):
            self.agent_pos = None
            self.stat_status.set("✓ Goal reached!")
            self.running = False
            self._refresh_all()
            return

        old_pos = self.agent_pos
        self.agent_pos = self.agent_path[self.agent_idx]
        self.agent_idx += 1

        if old_pos:
            self._update_cell(*old_pos)
        self._update_cell(*self.agent_pos)

        if self.dynamic_mode.get() and self.agent_idx < len(self.agent_path):
            self._maybe_spawn_wall()

        aid = self.root.after(MOVE_MS, self._move_agent)
        self.anim_ids.append(aid)

    def _maybe_spawn_wall(self):
        if random.random() > SPAWN_P: return

        candidates = []
        for r in range(self.rows):
            for c in range(self.cols):
                pos = (r, c)
                if (self.grid[r][c] == 0 and
                    pos != self.start and
                    pos != self.goal and
                    pos != self.agent_pos and
                    pos not in self.path_cells):
                    candidates.append(pos)

        remaining_path = self.agent_path[self.agent_idx:]
        path_candidates = [p for p in remaining_path
                           if p not in (self.start, self.goal, self.agent_pos)
                           and self.grid[p[0]][p[1]] == 0]

        if path_candidates and random.random() < 0.6:
            pos = random.choice(path_candidates)
        elif candidates:
            pos = random.choice(candidates)
        else:
            return

        self.grid[pos[0]][pos[1]] = 2
        self._update_cell(*pos)

        if pos in set(self.agent_path[self.agent_idx:]):
            self._replan()

    def _replan(self):
        self._cancel_anims()
        self.running = True

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 2:
                    self.grid[r][c] = 1

        self.stat_status.set("⚡ Re-planning…")
        self.visited_cells  = set()
        self.frontier_cells = set()
        self.path_cells     = set()
        self._refresh_all()

        aid = self.root.after(50, lambda: self._run_search(self.agent_pos))
        self.anim_ids.append(aid)

if __name__ == "__main__":
    root = tk.Tk()
    app = PathfinderApp(root)
    root.mainloop()
