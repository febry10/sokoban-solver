// Sokoban solver for make.fun's sokoban-solver-01.
//
// Approach: the levels are a one-wide hallway with small rooms hanging off it
// through single door cells, and a box that leaves its room can never come
// back, so a level's push-optimum is the sum of its rooms' push-optima. This
// program finds the doors (articulation cells of the floor graph), solves
// every room on its own with a push-optimal A* search (dead-square and
// freeze-deadlock pruning, box-to-goal matching heuristic), and concatenates
// the room solutions with hallway walks (walks are not scored, only pushes).
// If the room decomposition does not apply, it falls back to searching the
// whole level, first optimally, then greedily: any complete legal answer
// beats none. All search budgets are counted in expanded nodes, never in
// wall-clock time, and every tie is broken by a fixed rule, so two runs of
// this program on the same input produce byte-identical output.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed direction order used everywhere (deterministic tie-breaking).
// OPP[d] is the direction opposite d in this order.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];
const OPP = [1, 0, 3, 2];

// ---------------------------------------------------------------------------
// Level parsing (mirrors the checker's reading of XSB text).
// ---------------------------------------------------------------------------
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const n = w * h;
  const wall = new Uint8Array(n).fill(1);
  const goal = new Uint8Array(n);
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  // Neighbour table; -1 where the neighbour would leave the board.
  const nbr = new Int32Array(n * 4).fill(-1);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * w + x;
      for (let d = 0; d < 4; d++) {
        const nx = x + DIRS[d].dx;
        const ny = y + DIRS[d].dy;
        if (nx >= 0 && nx < w && ny >= 0 && ny < h) nbr[i * 4 + d] = ny * w + nx;
      }
    }
  }
  return { w, h, n, wall, goal, boxes, player, nbr };
}

// ---------------------------------------------------------------------------
// Small deterministic helpers on the floor graph.
// ---------------------------------------------------------------------------

// Flood fill over floor cells avoiding blocked cells; returns the reached set
// as a stamp array plus the smallest reached index (the region's canonical
// representative). Scratch arrays are supplied by the caller.
function flood(level, start, blocked, stamp, stampVal, outList) {
  let rep = start;
  let head = 0;
  outList.length = 0;
  outList.push(start);
  stamp[start] = stampVal;
  while (head < outList.length) {
    const c = outList[head++];
    if (c < rep) rep = c;
    for (let d = 0; d < 4; d++) {
      const nb = level.nbr[c * 4 + d];
      if (nb < 0 || level.wall[nb] || blocked[nb] || stamp[nb] === stampVal) continue;
      stamp[nb] = stampVal;
      outList.push(nb);
    }
  }
  return rep;
}

// BFS shortest walk from `from` to any cell of `targets` over floor cells
// avoiding blocked cells; returns the move letters, or null when unreachable.
function walkPath(level, from, targets, blocked) {
  if (targets.has(from)) return '';
  const prev = new Int32Array(level.n).fill(-2);
  const prevDir = new Int8Array(level.n).fill(-1);
  const queue = [from];
  prev[from] = -1;
  let head = 0;
  let found = -1;
  while (head < queue.length) {
    const c = queue[head++];
    if (targets.has(c)) { found = c; break; }
    for (let d = 0; d < 4; d++) {
      const nb = level.nbr[c * 4 + d];
      if (nb < 0 || level.wall[nb] || blocked[nb] || prev[nb] !== -2) continue;
      prev[nb] = c;
      prevDir[nb] = d;
      queue.push(nb);
    }
  }
  if (found < 0) return null;
  const letters = [];
  let c = found;
  while (c !== from) {
    letters.push(DIRS[prevDir[c]].ch);
    c = prev[c];
  }
  letters.reverse();
  return letters.join('');
}

// Articulation cells of the floor graph (iterative-order Tarjan, fixed order).
function articulationCells(level) {
  const n = level.n;
  const disc = new Int32Array(n).fill(-1);
  const low = new Int32Array(n);
  const isArt = new Uint8Array(n);
  let time = 0;
  const dfs = (root) => {
    // Explicit stack of [cell, parent, nextDir, childCount].
    const stack = [[root, -1, 0, 0]];
    disc[root] = low[root] = time++;
    while (stack.length) {
      const frame = stack[stack.length - 1];
      const c = frame[0];
      if (frame[2] < 4) {
        const d = frame[2]++;
        const nb = level.nbr[c * 4 + d];
        if (nb < 0 || level.wall[nb] || nb === frame[1]) continue;
        if (disc[nb] === -1) {
          disc[nb] = low[nb] = time++;
          frame[3]++;
          stack.push([nb, c, 0, 0]);
        } else {
          low[c] = Math.min(low[c], disc[nb]);
        }
      } else {
        stack.pop();
        const parent = frame[1];
        if (parent === -1) {
          if (frame[3] > 1) isArt[c] = 1;
        } else {
          const pf = stack[stack.length - 1];
          if (low[c] >= disc[parent]) isArt[parent] = 1;
          low[parent] = Math.min(low[parent], low[c]);
        }
      }
    }
  };
  for (let i = 0; i < n; i++) {
    if (!level.wall[i] && disc[i] === -1) dfs(i);
  }
  return isArt;
}

// BFS distances over the floor graph from `src`, avoiding blocked cells.
function bfsDistances(level, src, blocked) {
  const dist = new Int32Array(level.n).fill(-1);
  const queue = [src];
  dist[src] = 0;
  let head = 0;
  while (head < queue.length) {
    const c = queue[head++];
    for (let d = 0; d < 4; d++) {
      const nb = level.nbr[c * 4 + d];
      if (nb < 0 || level.wall[nb] || blocked[nb] || dist[nb] !== -1) continue;
      dist[nb] = dist[c] + 1;
      queue.push(nb);
    }
  }
  return dist;
}

// ---------------------------------------------------------------------------
// Room segmentation. A door is an articulation cell whose removal splits off
// a small region holding as many boxes as goals. Doors are tried in order of
// increasing BFS distance from the player, so a room's own door is always
// claimed before any narrower passage inside it, and a hallway cell whose far
// side holds more than one room (too many boxes / cells) is skipped in
// favour of the rooms' real doors further along.
// ---------------------------------------------------------------------------
const ROOM_MAX_CELLS = 50;
const ROOM_MAX_BOXES = 6;

function segmentRooms(level) {
  const n = level.n;
  const isArt = articulationCells(level);
  const noBlock = new Uint8Array(n);
  const dist = bfsDistances(level, level.player, noBlock);
  const candidates = [];
  for (let i = 0; i < n; i++) if (isArt[i] && i !== level.player && dist[i] >= 0) candidates.push(i);
  candidates.sort((a, b) => (dist[a] - dist[b]) || (a - b));

  const claimed = new Uint8Array(n);
  const boxAt = new Uint8Array(n);
  for (const b of level.boxes) boxAt[b] = 1;
  const rooms = [];

  const stamp = new Int32Array(n).fill(0);
  let stampVal = 0;
  const list = [];

  for (const door of candidates) {
    if (claimed[door]) continue;
    // Components of the floor graph without the door and without claimed
    // cells; the component holding the player is the hallway side.
    const blocked = new Uint8Array(n);
    for (let i = 0; i < n; i++) if (claimed[i] || i === door) blocked[i] = 1;
    const seen = new Uint8Array(n);
    for (let s = 0; s < n; s++) {
      if (level.wall[s] || blocked[s] || seen[s]) continue;
      stampVal++;
      flood(level, s, blocked, stamp, stampVal, list);
      for (const c of list) seen[c] = 1;
      if (list.includes(level.player)) continue;
      // A candidate room interior: count boxes and goals, door included.
      let boxes = boxAt[door] ? 1 : 0;
      let goals = level.goal[door] ? 1 : 0;
      for (const c of list) {
        if (boxAt[c]) boxes++;
        if (level.goal[c]) goals++;
      }
      if (boxes < 1 || boxes > ROOM_MAX_BOXES || boxes !== goals) continue;
      if (list.length + 1 > ROOM_MAX_CELLS) continue;
      const cells = [...list, door].sort((a, b) => a - b);
      for (const c of cells) claimed[c] = 1;
      rooms.push({ cells, door });
    }
  }
  // Success only when every box and every goal sits inside some room.
  let ok = rooms.length > 0;
  for (const b of level.boxes) if (!claimed[b]) ok = false;
  for (let i = 0; i < n; i++) if (level.goal[i] && !claimed[i]) ok = false;
  if (!ok) return null;
  // Fill in each room's boxes and goals.
  for (const room of rooms) {
    const inRoom = new Uint8Array(n);
    for (const c of room.cells) inRoom[c] = 1;
    room.inRoom = inRoom;
    room.boxes = level.boxes.filter((b) => inRoom[b]);
    room.goals = room.cells.filter((c) => level.goal[c]);
  }
  return rooms;
}

// ---------------------------------------------------------------------------
// Push search over one region (a room, or the whole level as a fallback).
// A* over pushes: a state is the sorted box cells plus the canonical
// representative of the player's reachable region. The heuristic is the
// optimal box-to-goal matching cost over precomputed single-box push
// distances (admissible), or the sum of per-box minima when there are many
// boxes. Boxes may only rest on boxAllowed cells; the player may stand on
// any playerFloor cell. Returns the move string, or null within the budget.
// ---------------------------------------------------------------------------
const INF = 1e9;

function solveRegion(level, opts) {
  const { playerFloor, boxAllowed, goals, startBoxes, playerStart, budget, weight } = opts;
  const n = level.n;
  const goalSet = new Uint8Array(n);
  for (const g of goals) goalSet[g] = 1;

  // Single-box push distances to each goal, by reverse pull-BFS: a box on y
  // was pushed there from x = y - d by a player standing on x - d.
  const dists = goals.map((g) => {
    const dist = new Int32Array(n).fill(-1);
    dist[g] = 0;
    const queue = [g];
    let head = 0;
    while (head < queue.length) {
      const y = queue[head++];
      for (let d = 0; d < 4; d++) {
        const back = level.nbr[y * 4 + OPP[d]]; // y - d
        if (back < 0 || !boxAllowed[back] || dist[back] !== -1) continue;
        const stand = level.nbr[back * 4 + OPP[d]]; // back - d
        if (stand < 0 || !playerFloor[stand]) continue;
        dist[back] = dist[y] + 1;
        queue.push(back);
      }
    }
    return dist;
  });
  // Dead squares: a box resting there (off goal) can never reach any goal.
  const dead = new Uint8Array(n);
  const minDist = new Int32Array(n).fill(-1);
  for (let c = 0; c < n; c++) {
    if (!boxAllowed[c]) continue;
    let best = -1;
    for (let j = 0; j < goals.length; j++) {
      const v = dists[j][c];
      if (v !== -1 && (best === -1 || v < best)) best = v;
    }
    minDist[c] = best;
    if (best === -1 && !goalSet[c]) dead[c] = 1;
  }
  for (const b of startBoxes) if (dead[b]) return null;

  // Freeze-deadlock test: a box is immovable when every push direction is
  // closed by a wall (or a cell outside the allowed sets) or by a box
  // already known immovable. Immovability is computed as a
  // fixpoint over the state's boxes; boxes that might still move away are
  // always treated as open, so the test only fires on boxes that provably
  // can never move again. A state holding an immovable box off its goal is
  // dead. Only the pushed box and its new neighbours can newly freeze, so
  // the caller runs this only when the pushed box lands next to a wall or
  // another box.
  const frozenScratch = new Uint8Array(n);
  // A box is immovable when no push direction is open, where a direction is
  // open when its target and stand cells are inside the allowed sets and not
  // occupied by an already-immovable box (a merely occupied cell counts as
  // open: that box may still move away).
  const immovable = (b, occ) => {
    for (let d = 0; d < 4; d++) {
      const target = level.nbr[b * 4 + d];
      const stand = level.nbr[b * 4 + OPP[d]];
      if (target < 0 || stand < 0) continue;
      if (!boxAllowed[target] || !playerFloor[stand]) continue;
      if (occ[target] && frozenScratch[target]) continue;
      if (occ[stand] && frozenScratch[stand]) continue;
      return false;
    }
    return true;
  };
  const hasFrozenOffGoal = (boxes, occ) => {
    for (const b of boxes) frozenScratch[b] = 0;
    let changed = true;
    while (changed) {
      changed = false;
      for (const b of boxes) {
        if (!frozenScratch[b] && immovable(b, occ)) {
          frozenScratch[b] = 1;
          changed = true;
        }
      }
    }
    return boxes.some((b) => frozenScratch[b] && !goalSet[b]);
  };
  const touchesBlocker = (cell, occ) => {
    for (let d = 0; d < 4; d++) {
      const nb = level.nbr[cell * 4 + d];
      if (nb < 0 || !boxAllowed[nb] || occ[nb]) return true;
    }
    return false;
  };

  const nBoxes = startBoxes.length;
  const useMatching = nBoxes <= 6 && goals.length <= 6;
  const heuristic = (boxes) => {
    if (!useMatching) {
      let sum = 0;
      for (const b of boxes) {
        const v = minDist[b];
        if (v === -1) return INF;
        sum += v;
      }
      return sum;
    }
    // DP over goal subsets: dp[mask] = cheapest assignment of the first
    // popcount(mask) boxes to the goals in mask.
    const m = goals.length;
    const dp = new Float64Array(1 << m).fill(INF);
    dp[0] = 0;
    for (let mask = 0; mask < (1 << m); mask++) {
      if (dp[mask] === INF) continue;
      let i = 0;
      for (let bits = mask; bits; bits &= bits - 1) i++;
      if (i >= nBoxes) continue;
      for (let j = 0; j < m; j++) {
        if (mask & (1 << j)) continue;
        const cost = dists[j][boxes[i]];
        if (cost === -1) continue;
        const nm = mask | (1 << j);
        if (dp[mask] + cost < dp[nm]) dp[nm] = dp[mask] + cost;
      }
    }
    return dp[(1 << m) - 1];
  };

  // Scratch buffers for flood fills.
  const stamp = new Int32Array(n).fill(0);
  let stampVal = 0;
  const floodList = [];
  const boxOcc = new Uint8Array(n);
  const regionRep = (playerCell, boxes) => {
    boxOcc.fill(0);
    for (const b of boxes) boxOcc[b] = 1;
    stampVal++;
    return flood(level, playerCell, boxOcc, stamp, stampVal, floodList);
  };
  const reachable = (playerCell, boxes) => {
    boxOcc.fill(0);
    for (const b of boxes) boxOcc[b] = 1;
    stampVal++;
    flood(level, playerCell, boxOcc, stamp, stampVal, floodList);
    const reach = new Uint8Array(n);
    for (const c of floodList) reach[c] = 1;
    return reach;
  };

  const keyOf = (boxes, rep) => {
    if (nBoxes <= 5) {
      let k = rep;
      for (let i = nBoxes - 1; i >= 0; i--) k = k * 256 + boxes[i];
      return k;
    }
    return boxes.join(',') + '|' + rep;
  };

  // Node store; the heap orders by (f, deeper g first, insertion order).
  const nodes = [];
  const bestG = new Map();
  const heap = [];
  const heapLess = (a, b) => {
    const na = nodes[a];
    const nb = nodes[b];
    if (na.f !== nb.f) return na.f < nb.f;
    if (na.g !== nb.g) return na.g > nb.g;
    return a < b;
  };
  const heapPush = (idx) => {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
        if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
        if (m === i) break;
        [heap[i], heap[m]] = [heap[m], heap[i]];
        i = m;
      }
    }
    return top;
  };

  // A hard cap on stored states bounds memory on pathological searches; it
  // is a work counter like the expansion budget, so runs stay deterministic.
  const nodeCap = opts.nodeCap ?? 600000;
  const startRep = regionRep(playerStart, startBoxes);
  const h0 = heuristic(startBoxes);
  if (h0 >= INF) return null;
  nodes.push({
    boxes: [...startBoxes], rep: startRep, g: 0, f: weight * h0,
    parent: -1, boxFrom: -1, dir: -1, playerCell: playerStart,
  });
  bestG.set(keyOf(startBoxes, startRep), 0);
  heapPush(0);

  let expansions = 0;
  let goalNode = -1;
  while (heap.length) {
    const idx = heapPop();
    const node = nodes[idx];
    if (node.g > (bestG.get(keyOf(node.boxes, node.rep)) ?? -1)) continue;
    // Goal test at pop time: every box on a goal, plus the caller's extra
    // condition (for rooms: the player must still be able to walk out).
    let solved = true;
    for (const b of node.boxes) if (!goalSet[b]) { solved = false; break; }
    if (solved && (!opts.mustExit || opts.exitOk(node.playerCell, node.boxes))) {
      goalNode = idx;
      break;
    }
    if (expansions >= budget) return null;
    expansions++;
    opts.counter.used++;
    const reach = reachable(node.playerCell, node.boxes);
    const occ = new Uint8Array(n);
    for (const b of node.boxes) occ[b] = 1;
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const stand = level.nbr[b * 4 + OPP[d]];
        const target = level.nbr[b * 4 + d];
        if (stand < 0 || target < 0) continue;
        if (!reach[stand]) continue;
        if (!boxAllowed[target] || occ[target] || dead[target]) continue;
        const childBoxes = [...node.boxes];
        childBoxes[bi] = target;
        childBoxes.sort((x, y) => x - y);
        if (!goalSet[target] && touchesBlocker(target, occ)) {
          const childOcc = new Uint8Array(n);
          for (const cb of childBoxes) childOcc[cb] = 1;
          if (hasFrozenOffGoal(childBoxes, childOcc)) continue;
        }
        const childRep = regionRep(b, childBoxes);
        const key = keyOf(childBoxes, childRep);
        const g = node.g + 1;
        if (bestG.has(key) && bestG.get(key) <= g) continue;
        const h = heuristic(childBoxes);
        if (h >= INF) continue;
        if (nodes.length >= nodeCap) return null;
        bestG.set(key, g);
        nodes.push({
          boxes: childBoxes, rep: childRep, g, f: g + weight * h,
          parent: idx, boxFrom: b, dir: d, playerCell: b,
        });
        heapPush(nodes.length - 1);
      }
    }
  }
  if (goalNode < 0) return null;

  // Reconstruct: walk the parent chain, then re-emit walks + pushes.
  const pushes = [];
  for (let i = goalNode; nodes[i].parent !== -1; i = nodes[i].parent) {
    pushes.push({ boxFrom: nodes[i].boxFrom, dir: nodes[i].dir });
  }
  pushes.reverse();
  const boxesNow = [...startBoxes];
  const occNow = new Uint8Array(n);
  let player = playerStart;
  let moves = '';
  for (const p of pushes) {
    occNow.fill(0);
    for (const b of boxesNow) occNow[b] = 1;
    const stand = level.nbr[p.boxFrom * 4 + OPP[p.dir]];
    const walk = walkPath(level, player, new Set([stand]), occNow);
    if (walk === null) return null;
    moves += walk + DIRS[p.dir].ch;
    const target = level.nbr[p.boxFrom * 4 + p.dir];
    boxesNow[boxesNow.indexOf(p.boxFrom)] = target;
    boxesNow.sort((a, b) => a - b);
    player = p.boxFrom;
  }
  return { moves, pushes: pushes.length, endPlayer: player, endBoxes: boxesNow };
}

// ---------------------------------------------------------------------------
// Replay a move string on a level (the same rules as the checker).
// ---------------------------------------------------------------------------
function replay(level, moves) {
  const box = new Uint8Array(level.n);
  for (const b of level.boxes) box[b] = 1;
  let player = level.player;
  let pushes = 0;
  for (const ch of moves) {
    const d = DIRS.findIndex((x) => x.ch === ch);
    const next = level.nbr[player * 4 + d];
    if (next < 0 || level.wall[next]) return { ok: false };
    if (box[next]) {
      const beyond = level.nbr[next * 4 + d];
      if (beyond < 0 || level.wall[beyond] || box[beyond]) return { ok: false };
      box[next] = 0;
      box[beyond] = 1;
      pushes++;
    }
    player = next;
  }
  let solved = true;
  for (let i = 0; i < level.n; i++) if (box[i] && !level.goal[i]) solved = false;
  return { ok: true, solved, pushes, player, box };
}

// ---------------------------------------------------------------------------
// Solve one level end to end.
// ---------------------------------------------------------------------------
const OPT_ROOM_BUDGET = 1200000;
const GREEDY_BUDGET = 300000;
const counter = { used: 0 };

function solveLevel(text) {
  const level = parseLevel(text);
  const allFloor = new Uint8Array(level.n);
  for (let i = 0; i < level.n; i++) if (!level.wall[i]) allFloor[i] = 1;

  const rooms = segmentRooms(level);
  if (rooms) {
    // Entry cells of a room: floor neighbours of its cells that belong to no
    // room (pure hallway cells the player can stand on to enter).
    const roomOf = new Int32Array(level.n).fill(-1);
    rooms.forEach((room, ri) => { for (const c of room.cells) roomOf[c] = ri; });
    for (const room of rooms) {
      const entries = new Set();
      for (const c of room.cells) {
        for (let d = 0; d < 4; d++) {
          const nb = level.nbr[c * 4 + d];
          if (nb >= 0 && !level.wall[nb] && roomOf[nb] === -1) entries.add(nb);
        }
      }
      room.entries = [...entries].sort((a, b) => a - b);
    }
    // Solve order: by the door's BFS distance from the player.
    const noBlock = new Uint8Array(level.n);
    const dist0 = bfsDistances(level, level.player, noBlock);
    const order = rooms.map((_, i) => i).sort((a, b) =>
      (dist0[rooms[a].door] - dist0[rooms[b].door]) || (rooms[a].door - rooms[b].door));

    const solveRoom = (room, mustExit) => {
      if (!room.entries.length) return null;
      const boxOcc = new Uint8Array(level.n);
      for (const b of level.boxes) boxOcc[b] = 1;
      const dist = bfsDistances(level, level.player, boxOcc);
      let entry = room.entries[0];
      for (const e of room.entries) if (dist[e] !== -1 && (dist[entry] === -1 || dist[e] < dist[entry])) entry = e;
      const playerFloor = new Uint8Array(level.n);
      for (const c of room.cells) playerFloor[c] = 1;
      for (const e of room.entries) playerFloor[e] = 1;
      const exitOk = (playerCell, boxes) => {
        const blocked = new Uint8Array(level.n);
        for (const b of boxes) blocked[b] = 1;
        // The player's own room boxes block; other rooms' boxes sit inside
        // their rooms and are not on the way out, but block them too for
        // exactness with the assembled board.
        for (const b of level.boxes) if (!room.inRoom[b]) blocked[b] = 1;
        const stamp = new Int32Array(level.n).fill(0);
        const list = [];
        flood(level, playerCell, blocked, stamp, 1, list);
        return list.some((c) => !room.inRoom[c]);
      };
      const base = {
        playerFloor, boxAllowed: room.inRoom, goals: room.goals,
        startBoxes: room.boxes, playerStart: entry, mustExit, exitOk, counter,
      };
      const sol = solveRegion(level, { ...base, budget: OPT_ROOM_BUDGET, weight: 1 })
        ?? solveRegion(level, { ...base, budget: GREEDY_BUDGET, weight: 3 });
      if (sol) sol.entry = entry;
      return sol;
    };

    // Solve every room with the exit condition; rooms that only solve
    // without it are solved last instead.
    const solutions = new Map();
    const deferred = [];
    for (const ri of order) {
      const sol = solveRoom(rooms[ri], true);
      if (sol) solutions.set(ri, sol);
      else deferred.push(ri);
    }
    for (const ri of deferred) {
      const sol = solveRoom(rooms[ri], false);
      if (sol) solutions.set(ri, sol);
    }
    if (solutions.size === rooms.length) {
      // Assemble: walk the hallway to each room's entry, then play its
      // solution, tracking the real board state as we go.
      const playOrder = [...order.filter((ri) => !deferred.includes(ri)), ...deferred];
      const state = { player: level.player, box: new Uint8Array(level.n) };
      for (const b of level.boxes) state.box[b] = 1;
      let moves = '';
      let failed = false;
      for (const ri of playOrder) {
        const room = rooms[ri];
        const sol = solutions.get(ri);
        // The room solution starts at its entry cell; find it again from the
        // solution's first walk by re-deriving: the entry used at solve time
        // is recomputed identically (same initial board), but boxes moved
        // since; choose the entry the solution expects: it is the entry cell
        // stored during solveRoom. We stored it on the solution object.
        const entry = sol.entry;
        const walk = walkPath(level, state.player, new Set([entry]), state.box);
        if (walk === null) { failed = true; break; }
        moves += walk;
        moves += sol.moves;
        // Apply the room moves to the tracked board.
        let p = entry;
        for (const ch of sol.moves) {
          const d = DIRS.findIndex((x) => x.ch === ch);
          const next = level.nbr[p * 4 + d];
          if (next < 0 || level.wall[next]) { failed = true; break; }
          if (state.box[next]) {
            const beyond = level.nbr[next * 4 + d];
            if (beyond < 0 || level.wall[beyond] || state.box[beyond]) { failed = true; break; }
            state.box[next] = 0;
            state.box[beyond] = 1;
          }
          p = next;
        }
        if (failed) break;
        state.player = p;
      }
      if (!failed) {
        const check = replay(level, moves);
        if (check.ok && check.solved) return moves;
      }
    }
  }

  // Fallback: search the whole level, optimally then greedily.
  const base = {
    playerFloor: allFloor, boxAllowed: allFloor,
    goals: levelCellsGoals(level),
    startBoxes: level.boxes, playerStart: level.player,
    mustExit: false, exitOk: () => true, counter,
  };
  const sol = solveRegion(level, { ...base, budget: 2500000, weight: 1 })
    ?? solveRegion(level, { ...base, budget: 500000, weight: 3 });
  if (sol) {
    const check = replay(level, sol.moves);
    if (check.ok && check.solved) return sol.moves;
  }
  return '';
}

function levelCellsGoals(level) {
  const goals = [];
  for (let i = 0; i < level.n; i++) if (level.goal[i]) goals.push(i);
  return goals;
}

// ---------------------------------------------------------------------------
function main() {
  for (const id of readdirSync(INPUT_DIR).sort()) {
    let moves = '';
    try {
      moves = solveLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'));
    } catch {
      moves = '';
    }
    mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
    writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
  }
}

main();
