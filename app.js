// Constant speed settings mapping directly to the original Python version
const ANIM_MS = 30;
const MOVE_MS = 120;
const SPAWN_P = 0.03;

// Min-Priority Queue (Binary Heap) Implementation for A* and GBFS
class MinHeap {
    constructor() {
        this.heap = [];
    }

    push(val) {
        this.heap.push(val);
        this._bubbleUp();
    }

    pop() {
        if (this.heap.length === 0) return null;
        const min = this.heap[0];
        const end = this.heap.pop();
        if (this.heap.length > 0) {
            this.heap[0] = end;
            this._sinkDown();
        }
        return min;
    }

    isEmpty() {
        return this.heap.length === 0;
    }

    _bubbleUp() {
        let idx = this.heap.length - 1;
        const element = this.heap[idx];
        while (idx > 0) {
            let parentIdx = Math.floor((idx - 1) / 2);
            let parent = this.heap[parentIdx];
            // Compare score first. If equal, compare counter to ensure FIFO-like stability
            if (element[0] > parent[0] || (element[0] === parent[0] && element[1] >= parent[1])) break;
            this.heap[parentIdx] = element;
            this.heap[idx] = parent;
            idx = parentIdx;
        }
    }

    _sinkDown() {
        let idx = 0;
        const length = this.heap.length;
        const element = this.heap[0];
        while (true) {
            let leftChildIdx = 2 * idx + 1;
            let rightChildIdx = 2 * idx + 2;
            let leftChild, rightChild;
            let swap = null;

            if (leftChildIdx < length) {
                leftChild = this.heap[leftChildIdx];
                if (leftChild[0] < element[0] || (leftChild[0] === element[0] && leftChild[1] < element[1])) {
                    swap = leftChildIdx;
                }
            }

            if (rightChildIdx < length) {
                rightChild = this.heap[rightChildIdx];
                if (swap === null) {
                    if (rightChild[0] < element[0] || (rightChild[0] === element[0] && rightChild[1] < element[1])) {
                        swap = rightChildIdx;
                    }
                } else {
                    if (rightChild[0] < leftChild[0] || (rightChild[0] === leftChild[0] && rightChild[1] < leftChild[1])) {
                        swap = rightChildIdx;
                    }
                }
            }

            if (swap === null) break;
            this.heap[idx] = this.heap[swap];
            this.heap[swap] = element;
            idx = swap;
        }
    }
}

// Heuristics Definitions
const HEURISTICS = {
    "Manhattan": (a, b) => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]),
    "Euclidean": (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]),
    "Chebyshev": (a, b) => Math.max(Math.abs(a[0] - b[0]), Math.abs(a[1] - b[1]))
};

// Movements DIRS
const DIRS_4 = [[-1, 0], [1, 0], [0, -1], [0, 1]];
const DIRS_8 = [[-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [-1, 1], [1, -1], [1, 1]];

// Pathfinding algorithms
function gbfs(grid, rows, cols, start, goal, hFn, dirs) {
    const startKey = `${start[0]},${start[1]}`;
    const startH = hFn(start, goal);
    const openHeap = new MinHeap();
    let counter = 0;
    openHeap.push([startH, counter, start]);

    const cameFrom = new Map();
    cameFrom.set(startKey, null);

    const visited = [];
    const frontierHistory = [];

    while (!openHeap.isEmpty()) {
        const item = openHeap.pop();
        const cur = item[2];

        visited.push(cur);

        const currentFrontier = [];
        for (const element of openHeap.heap) {
            currentFrontier.push(element[2]);
        }
        frontierHistory.push(currentFrontier);

        if (cur[0] === goal[0] && cur[1] === goal[1]) {
            return {
                path: reconstructPath(cameFrom, goal),
                visited,
                frontierHistory
            };
        }

        for (const [dr, dc] of dirs) {
            const nr = cur[0] + dr;
            const nc = cur[1] + dc;
            const nbKey = `${nr},${nc}`;

            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                if (grid[nr][nc] !== 1 && !cameFrom.has(nbKey)) {
                    cameFrom.set(nbKey, cur);
                    counter++;
                    openHeap.push([hFn([nr, nc], goal), counter, [nr, nc]]);
                }
            }
        }
    }

    return { path: [], visited, frontierHistory };
}

function astar(grid, rows, cols, start, goal, hFn, dirs) {
    const startKey = `${start[0]},${start[1]}`;
    const gCost = new Map();
    gCost.set(startKey, 0);

    const openHeap = new MinHeap();
    let counter = 0;
    openHeap.push([hFn(start, goal), counter, start]);

    const cameFrom = new Map();
    cameFrom.set(startKey, null);

    const closed = new Set();
    const visited = [];
    const frontierHistory = [];

    while (!openHeap.isEmpty()) {
        const item = openHeap.pop();
        const cur = item[2];
        const curKey = `${cur[0]},${cur[1]}`;

        if (closed.has(curKey)) continue;
        closed.add(curKey);
        visited.push(cur);

        const currentFrontier = [];
        for (const element of openHeap.heap) {
            const elKey = `${element[2][0]},${element[2][1]}`;
            if (!closed.has(elKey)) {
                currentFrontier.push(element[2]);
            }
        }
        frontierHistory.push(currentFrontier);

        if (cur[0] === goal[0] && cur[1] === goal[1]) {
            return {
                path: reconstructPath(cameFrom, goal),
                visited,
                frontierHistory
            };
        }

        const curGCost = gCost.get(curKey) || 0;

        for (const [dr, dc] of dirs) {
            const nr = cur[0] + dr;
            const nc = cur[1] + dc;
            const nbKey = `${nr},${nc}`;

            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                if (grid[nr][nc] !== 1 && !closed.has(nbKey)) {
                    const step = (Math.abs(dr) + Math.abs(dc) === 2) ? Math.hypot(dr, dc) : 1.0;
                    const tentative = curGCost + step;

                    const prevGCost = gCost.has(nbKey) ? gCost.get(nbKey) : Infinity;

                    if (tentative < prevGCost) {
                        gCost.set(nbKey, tentative);
                        cameFrom.set(nbKey, cur);
                        counter++;
                        openHeap.push([tentative + hFn([nr, nc], goal), counter, [nr, nc]]);
                    }
                }
            }
        }
    }

    return { path: [], visited, frontierHistory };
}

function reconstructPath(cameFrom, goal) {
    const path = [];
    let cur = goal;
    while (cur !== null) {
        path.push(cur);
        const curKey = `${cur[0]},${cur[1]}`;
        cur = cameFrom.get(curKey);
    }
    path.reverse();
    return path;
}

// App State
let rows = 20;
let cols = 28;
let grid = Array.from({ length: rows }, () => Array(cols).fill(0));
let start = [1, 1];
let goal = [rows - 2, cols - 2];

let editMode = 'wall';
let running = false;
let dynamicMode = false;
let diagonalMode = false;
let algo = 'A*';
let heuristic = 'Manhattan';
let density = 0.30;

let visitedCells = new Set();
let frontierCells = new Set();
let pathCells = new Set();
let agentPos = null;
let agentPath = [];
let agentIdx = 0;
let animTimers = [];

// Interaction details
let isMouseDown = false;
let drawVal = null;
let cellElements = []; // DOM cell references

// Selectors
const gridContainer = document.getElementById('grid-container');
const rowsInput = document.getElementById('spin-rows');
const colsInput = document.getElementById('spin-cols');
const densityInput = document.getElementById('slider-density');
const densityValDisplay = document.getElementById('density-val');
const generateBtn = document.getElementById('btn-generate');
const clearBtn = document.getElementById('btn-clear');
const findBtn = document.getElementById('btn-find');
const resetBtn = document.getElementById('btn-reset');
const stopBtn = document.getElementById('btn-stop');

const chkDynamic = document.getElementById('chk-dynamic');
const chkDiagonal = document.getElementById('chk-diagonal');

const metricVisited = document.getElementById('metric-visited');
const metricCost = document.getElementById('metric-cost');
const metricTime = document.getElementById('metric-time');
const statusText = document.getElementById('status-text');

// Initialize events
document.addEventListener('mouseup', () => {
    isMouseDown = false;
    drawVal = null;
});

// Edit mode radios
const modeButtons = document.querySelectorAll('.radio-btn');
modeButtons.forEach(btn => {
    const radio = btn.querySelector('input');
    btn.addEventListener('click', () => {
        modeButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        radio.checked = true;
        editMode = radio.value;
    });
});

// Algorithm radios
document.querySelectorAll('input[name="algo"]').forEach(radio => {
    radio.addEventListener('change', (e) => {
        algo = e.target.value;
    });
});

// Heuristics radios
document.querySelectorAll('input[name="heuristic"]').forEach(radio => {
    radio.addEventListener('change', (e) => {
        heuristic = e.target.value;
    });
});

// Checkboxes
chkDynamic.addEventListener('change', (e) => {
    dynamicMode = e.target.checked;
});
chkDiagonal.addEventListener('change', (e) => {
    diagonalMode = e.target.checked;
});

// Density slider
densityInput.addEventListener('input', (e) => {
    density = parseFloat(e.target.value);
    densityValDisplay.textContent = density.toFixed(2);
});

// Action Buttons
generateBtn.addEventListener('click', generateMap);
clearBtn.addEventListener('click', clearMap);
findBtn.addEventListener('click', startSearch);
resetBtn.addEventListener('click', resetVisualisation);
stopBtn.addEventListener('click', stopSearch);

// Functions
function updateCellClass(r, c) {
    const el = cellElements[r][c];
    if (!el) return;

    el.className = 'cell';

    const isStart = r === start[0] && c === start[1];
    const isGoal = r === goal[0] && c === goal[1];
    const isAgent = agentPos && r === agentPos[0] && c === agentPos[1];
    const key = `${r},${c}`;

    if (isStart) {
        el.classList.add('start');
    } else if (isGoal) {
        el.classList.add('goal');
    } else if (isAgent) {
        el.classList.add('agent');
    } else if (grid[r][c] === 2) {
        el.classList.add('new-wall');
    } else if (grid[r][c] === 1) {
        el.classList.add('wall');
    } else if (pathCells.has(key)) {
        el.classList.add('path');
    } else if (frontierCells.has(key)) {
        el.classList.add('frontier');
    } else if (visitedCells.has(key)) {
        el.classList.add('visited');
    }
}

function refreshAllCells() {
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            updateCellClass(r, c);
        }
    }
}

function buildGridDOM() {
    gridContainer.innerHTML = '';
    gridContainer.style.gridTemplateRows = `repeat(${rows}, 26px)`;
    gridContainer.style.gridTemplateColumns = `repeat(${cols}, 26px)`;
    
    cellElements = Array.from({ length: rows }, () => Array(cols).fill(null));

    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const cell = document.createElement('div');
            cell.className = 'cell';
            
            // Mouse event handlers for drawing and dragging nodes
            cell.addEventListener('mousedown', (e) => {
                e.preventDefault();
                if (running) return;
                isMouseDown = true;
                
                const isStart = r === start[0] && c === start[1];
                const isGoal = r === goal[0] && c === goal[1];

                if (editMode === 'wall') {
                    if (isStart || isGoal) return;
                    drawVal = grid[r][c] === 1 ? 0 : 1;
                    grid[r][c] = drawVal;
                    updateCellClass(r, c);
                } else if (editMode === 'start') {
                    if (isGoal) return;
                    const oldStart = [...start];
                    start = [r, c];
                    grid[r][c] = 0;
                    updateCellClass(oldStart[0], oldStart[1]);
                    updateCellClass(r, c);
                } else if (editMode === 'goal') {
                    if (isStart) return;
                    const oldGoal = [...goal];
                    goal = [r, c];
                    grid[r][c] = 0;
                    updateCellClass(oldGoal[0], oldGoal[1]);
                    updateCellClass(r, c);
                }
            });

            cell.addEventListener('mouseenter', () => {
                if (!isMouseDown || running) return;

                const isStart = r === start[0] && c === start[1];
                const isGoal = r === goal[0] && c === goal[1];

                if (editMode === 'wall') {
                    if (isStart || isGoal) return;
                    if (drawVal !== null) {
                        grid[r][c] = drawVal;
                        updateCellClass(r, c);
                    }
                } else if (editMode === 'start') {
                    if (isGoal) return;
                    const oldStart = [...start];
                    start = [r, c];
                    grid[r][c] = 0;
                    updateCellClass(oldStart[0], oldStart[1]);
                    updateCellClass(r, c);
                } else if (editMode === 'goal') {
                    if (isStart) return;
                    const oldGoal = [...goal];
                    goal = [r, c];
                    grid[r][c] = 0;
                    updateCellClass(oldGoal[0], oldGoal[1]);
                    updateCellClass(r, c);
                }
            });

            cellElements[r][c] = cell;
            gridContainer.appendChild(cell);
            updateCellClass(r, c);
        }
    }
}

function updateGridDimensions() {
    const reqRows = parseInt(rowsInput.value) || 20;
    const reqCols = parseInt(colsInput.value) || 28;
    
    rows = Math.max(5, Math.min(40, reqRows));
    cols = Math.max(5, Math.min(60, reqCols));
    
    rowsInput.value = rows;
    colsInput.value = cols;

    // Bounds adjustments for start and goal
    start[0] = Math.min(start[0], rows - 1);
    start[1] = Math.min(start[1], cols - 1);
    goal[0] = Math.min(goal[0], rows - 1);
    goal[1] = Math.min(goal[1], cols - 1);

    if (start[0] === goal[0] && start[1] === goal[1]) {
        if (goal[0] > 0) {
            start = [goal[0] - 1, goal[1]];
        } else if (goal[0] < rows - 1) {
            start = [goal[0] + 1, goal[1]];
        }
    }
}

function generateMap() {
    if (running) return;
    stopSearch();
    updateGridDimensions();
    
    grid = Array.from({ length: rows }, () => Array(cols).fill(0));
    
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const isStart = r === start[0] && c === start[1];
            const isGoal = r === goal[0] && c === goal[1];
            if (!isStart && !isGoal) {
                grid[r][c] = Math.random() < density ? 1 : 0;
            }
        }
    }
    
    resetVisualisationState();
    buildGridDOM();
}

function clearMap() {
    if (running) return;
    stopSearch();
    updateGridDimensions();

    grid = Array.from({ length: rows }, () => Array(cols).fill(0));
    resetVisualisationState();
    buildGridDOM();
}

function resetVisualisationState() {
    cancelAnimations();
    visitedCells.clear();
    frontierCells.clear();
    pathCells.clear();
    agentPos = null;
    agentPath = [];
    agentIdx = 0;
    
    metricVisited.textContent = "0";
    metricCost.textContent = "0";
    metricTime.textContent = "0 ms";
    statusText.textContent = "Ready";
}

function resetVisualisation() {
    cancelAnimations();
    visitedCells.clear();
    frontierCells.clear();
    pathCells.clear();
    agentPos = null;
    agentPath = [];
    agentIdx = 0;

    // Convert new walls (2) back to regular walls (1)
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            if (grid[r][c] === 2) {
                grid[r][c] = 1;
            }
        }
    }

    metricVisited.textContent = "0";
    metricCost.textContent = "0";
    metricTime.textContent = "0 ms";
    statusText.textContent = "Ready";
    refreshAllCells();
}

function cancelAnimations() {
    animTimers.forEach(t => clearTimeout(t));
    animTimers = [];
    running = false;
}

function stopSearch() {
    cancelAnimations();
    statusText.textContent = "Stopped";
}

function startSearch() {
    if (running) return;
    resetVisualisation();
    runSearch(start);
}

function runSearch(fromNode) {
    const hFn = HEURISTICS[heuristic];
    const dirs = diagonalMode ? DIRS_8 : DIRS_4;

    const t0 = performance.now();
    let result;
    if (algo === 'A*') {
        result = astar(grid, rows, cols, fromNode, goal, hFn, dirs);
    } else {
        result = gbfs(grid, rows, cols, fromNode, goal, hFn, dirs);
    }
    const elapsed = performance.now() - t0;

    metricTime.textContent = `${elapsed.toFixed(1)} ms`;
    metricVisited.textContent = result.visited.length.toString();

    if (result.path.length === 0) {
        statusText.textContent = "No path found!";
        alert("No path exists between Start/Agent and Goal.");
        running = false;
        return;
    }

    let cost = 0;
    for (let i = 1; i < result.path.length; i++) {
        const dr = result.path[i][0] - result.path[i - 1][0];
        const dc = result.path[i][1] - result.path[i - 1][1];
        cost += (Math.abs(dr) + Math.abs(dc) === 2) ? Math.hypot(dr, dc) : 1.0;
    }

    metricCost.textContent = cost.toFixed(1);
    statusText.textContent = `Animating (${algo})…`;
    running = true;

    animateSearch(result.visited, result.frontierHistory, result.path);
}

function animateSearch(visited, frontiers, path) {
    let step = 0;

    function tick() {
        if (!running) return;

        if (step < visited.length) {
            const v = visited[step];
            const isStart = v[0] === start[0] && v[1] === start[1];
            const isGoal = v[0] === goal[0] && v[1] === goal[1];

            if (!isStart && !isGoal) {
                visitedCells.add(`${v[0]},${v[1]}`);
                updateCellClass(v[0], v[1]);
            }

            if (step < frontiers.length) {
                const oldFrontier = new Set(frontierCells);
                frontierCells.clear();

                for (const cell of frontiers[step]) {
                    const key = `${cell[0]},${cell[1]}`;
                    const isCellStart = cell[0] === start[0] && cell[1] === start[1];
                    const isCellGoal = cell[0] === goal[0] && cell[1] === goal[1];

                    if (!visitedCells.has(key) && !isCellStart && !isCellGoal) {
                        frontierCells.add(key);
                    }
                }

                // Update only cells that changed frontier state
                const union = new Set([...oldFrontier, ...frontierCells]);
                for (const key of union) {
                    const [r, c] = key.split(',').map(Number);
                    updateCellClass(r, c);
                }
            }

            step++;
            const tid = setTimeout(tick, ANIM_MS);
            animTimers.push(tid);
        } else {
            frontierCells.clear();
            pathCells.clear();
            for (const p of path) {
                pathCells.add(`${p[0]},${p[1]}`);
            }
            refreshAllCells();
            statusText.textContent = "Path found! Moving agent…";

            agentPath = path;
            agentIdx = 0;
            const tid = setTimeout(moveAgent, 300);
            animTimers.push(tid);
        }
    }

    tick();
}

function moveAgent() {
    if (!running) return;

    if (agentIdx >= agentPath.length) {
        agentPos = null;
        statusText.textContent = "✓ Goal reached!";
        running = false;
        refreshAllCells();
        return;
    }

    const oldPos = agentPos;
    agentPos = agentPath[agentIdx];
    agentIdx++;

    if (oldPos) {
        updateCellClass(oldPos[0], oldPos[1]);
    }
    updateCellClass(agentPos[0], agentPos[1]);

    if (dynamicMode && agentIdx < agentPath.length) {
        maybeSpawnWall();
    }

    const tid = setTimeout(moveAgent, MOVE_MS);
    animTimers.push(tid);
}

function maybeSpawnWall() {
    if (Math.random() > SPAWN_P) return;

    const candidates = [];
    const pathKeys = new Set(agentPath.map(p => `${p[0]},${p[1]}`));

    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const isStart = r === start[0] && c === start[1];
            const isGoal = r === goal[0] && c === goal[1];
            const isAgent = agentPos && r === agentPos[0] && c === agentPos[1];
            const key = `${r},${c}`;

            if (grid[r][c] === 0 && !isStart && !isGoal && !isAgent && !pathKeys.has(key)) {
                candidates.push([r, c]);
            }
        }
    }

    const remainingPath = agentPath.slice(agentIdx);
    const pathCandidates = remainingPath.filter(p => {
        const isStart = p[0] === start[0] && p[1] === start[1];
        const isGoal = p[0] === goal[0] && p[1] === goal[1];
        const isAgent = agentPos && p[0] === agentPos[0] && p[1] === agentPos[1];
        return !isStart && !isGoal && !isAgent && grid[p[0]][p[1]] === 0;
    });

    let selectedPos = null;
    if (pathCandidates.length > 0 && Math.random() < 0.6) {
        selectedPos = pathCandidates[Math.floor(Math.random() * pathCandidates.length)];
    } else if (candidates.length > 0) {
        selectedPos = candidates[Math.floor(Math.random() * candidates.length)];
    } else {
        return;
    }

    const [pr, pc] = selectedPos;
    grid[pr][pc] = 2; // Newly spawned obstacle status
    updateCellClass(pr, pc);

    // If spawned wall lies on remaining path, trigger replanning
    const onRemainingPath = remainingPath.some(p => p[0] === pr && p[1] === pc);
    if (onRemainingPath) {
        replan();
    }
}

function replan() {
    cancelAnimations();
    running = true;

    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            if (grid[r][c] === 2) {
                grid[r][c] = 1;
            }
        }
    }

    statusText.textContent = "⚡ Re-planning…";
    visitedCells.clear();
    frontierCells.clear();
    pathCells.clear();
    refreshAllCells();

    const tid = setTimeout(() => {
        runSearch(agentPos);
    }, 50);
    animTimers.push(tid);
}

// Initial draw
buildGridDOM();
