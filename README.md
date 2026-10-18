# WAAM-HyCoS-
Hybrid geometry optimization and collision-aware scheduling for multi-robot WAAM using Greedy, ALNS, and CP-SAT.

# WAAM-HyCoS

### Hybrid Geometry and Conflict-aware Scheduling Optimization for Multi-Robot WAAM

WAAM-HyCoS is an independent optimization solver developed for cooperative Wire Arc Additive Manufacturing (WAAM) using three industrial robots.

The solver generates deposition trajectories directly from STL geometry and combines geometry-aware task partitioning, collision-aware scheduling, Adaptive Large Neighborhood Search (ALNS), and local CP-SAT optimization to minimize makespan while satisfying geometric accuracy, robot reachability, and collision constraints.

## Key Features

- **Geometry Optimization:** STL slicing, Contour/Raster/Centerline toolpath generation, Even-Odd boundary reconstruction, and adaptive deposition path repair.
- **Task Partitioning:** Weighted Voronoi-based spatial task allocation and robot reachability analysis.
- **Local Support DAG:** Task-level precedence scheduling to reduce unnecessary layer synchronization.
- **Collision-aware Scheduling:** Greedy initial scheduling with continuous-time TCP and arm-envelope collision checking.
- **Hybrid Optimization:** Seven ALNS improvement operators, including local CP-SAT optimization.
- **Validation:** Trajectory CSV generation and preflight checks compatible with WAAM Validator v1.0.1.

## Validation Results

The independently generated trajectories passed all six public validation cases using WAAM Validator v1.0.1.

| Model | Makespan (s) | Validation |
|---|---:|---|
| 01 Honeycomb | 1,528.94 | PASS |
| 02 Spiral | 2,874.25 | PASS |
| 03 Triangular truss | 2,364.06 | PASS |
| 04 Motor inner | 72,865.85 | PASS |
| 05 Twisted shell | 234,936.33 | PASS |
| 06 Angle frame | 7,721.85 | PASS |

These results represent the validated baseline trajectories. Improvements from the optional ALNS/CP-SAT optimization mode require separate benchmarking.

## Execution

Generate trajectories for all six public cases:

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 2 3 4 5 6 --mode generate
```

Run the optimization mode:

```powershell
.\rl_project\.venv\Scripts\python.exe -m rl_project.solve --ids 1 2 3 4 5 6 --mode optimize
```

Run the official validator on a generated job:

```powershell
.\rl_project\.venv\Scripts\python.exe -m waam_validator.cli run .\rl_project\results\01 --json
```

## Development Notes

- Language: Python
- Algorithms: Geometry Optimization, Greedy Scheduling, ALNS, Local CP-SAT
- Reinforcement Learning: Not used
- Validator: WAAM Validator v1.0.1
- Custom solver location: `rl_project/`
- Original validator source code and public test inputs are preserved without modification.

The solver is developed independently of other students' submission solvers. The WAAM Validator is used as the validation framework; its original authorship and license should be acknowledged separately.

## Project Status

- Independent trajectory generation implemented
- Six public validation cases passed
- ALNS and local CP-SAT implemented
- Makespan optimization benchmarking remains future work
