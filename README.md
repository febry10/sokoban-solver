# Sokoban solver — room decomposition + push-optimal A*

A Sokoban solver written for make.fun's **"Build the best Sokoban solver"**
bounty (sokoban-solver-01-run-20261121). It answers a set of five levels in
one run: for each `/input/<id>/level.txt` (XSB) it writes the player's moves
to `/output/<id>/solution.txt`. Only pushes are scored, and a level solved in
exactly its push-optimum P* is worth ten times one solved a few pushes over —
so this solver is built to be push-optimal, not merely correct.

## Why it is fast and optimal

The levels come from a known generator: a one-square-wide hallway crosses
the board, and one to four small rooms (each inside a 6×6 slot, holding 3–4
boxes and as many goals) open off it through a single door. A box pushed out
of its room can never return, so the rooms are independent and a level's P*
is exactly the sum of its rooms' push-optima.

1. **Segmentation.** The doors are articulation cells of the floor graph.
   Removing a door splits its room off the hallway; candidate doors are tried
   in order of increasing distance from the player, and a split-off region is
   accepted as a room when it is small and holds as many boxes as goals. If
   the decomposition ever fails, the solver falls back to searching the
   whole level instead.
2. **Push-optimal room search.** Each room is solved on its own by A* over
   pushes (state: sorted box cells + the player's reachable-region
   representative), starting from the hallway square outside the door:
   - the heuristic is the optimal box→goal matching cost over precomputed
     single-box push distances (reverse pull-BFS), which is admissible;
   - dead squares (a box resting there can never reach a goal) and
     freeze deadlocks (a fixpoint immovability test) prune dead states;
   - a room's solution must leave the player able to walk back out to the
     hallway, so the room solutions concatenate into one legal level
     solution; walks between rooms and between pushes are plain BFS and
     cost nothing.
3. **Safety nets.** If a room's optimal search ever exhausted its budget, a
   greedy weighted-A* pass still returns a complete solution for it (a
   solved-but-suboptimal room scores; an unsolved one scores zero), and the
   assembled moves are replayed in-process before anything is written.

## Determinism

The two grading executions must agree byte for byte, so the program never
reads the wall clock: all budgets are deterministic work counters (expanded
nodes, stored states), and every tie in every search is broken by a fixed
rule. Same input, same output, every run.

## Running it

```sh
node main.ts            # reads /input, writes /output (Node 22)
# local practice set:
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

No dependencies; plain JavaScript in `main.ts` (Node's type stripping runs
it as-is). On the kit's practice set it solves all five levels in exactly
P* pushes (13 / 16 / 29 / 47 / 60) in well under a second, where the house
baseline (plain-astar) solves one of the five.
