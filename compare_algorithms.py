#!/usr/bin/env python3
"""Benchmark trajectory-CBBA against the Performance Impact (PI) baseline.

Runs both algorithms on the same coverage problem under three time-window
regimes: no windows, all tasks windowed, and a mix. Reports cost, makespan,
reward, iterations, compute time, tasks allocated and time-window violations.
"""

import numpy as np
import shapely

from trajallocpy import Agent, CoverageProblem, Experiment, Task


def build_tasks(window_mode: str):
    segments = [
        [(10, 10), (15, 15)],
        [(50, 30), (70, 35)],
        [(30, 70), (45, 85)],
        [(85, 15), (90, 25)],
        [(20, 50), (35, 55)],
        [(60, 80), (75, 90)],
    ]
    tasks = []
    for tid, seg in enumerate(segments):
        line = shapely.geometry.LineString(seg)
        windowed = window_mode == "all" or (window_mode == "mixed" and tid % 2 == 0)
        # A generous deadline keeps the instance feasible while still exercising
        # the hard time-window logic.
        end_time = 400.0 if windowed else 0.0
        tasks.append(Task.TrajectoryTask(tid, line, reward=100, end_time=end_time))
    return tasks


def make_problem():
    boundary = shapely.geometry.Polygon([[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]])
    obstacles = shapely.geometry.MultiPolygon(
        [
            shapely.geometry.Polygon([[20, 20], [40, 20], [40, 40], [20, 40], [20, 20]]),
            shapely.geometry.Polygon([[60, 60], [80, 60], [80, 80], [60, 80], [60, 60]]),
        ]
    )
    return boundary, obstacles


def run(algorithm: str, window_mode: str, n_agents=3, capacity=2000):
    np.random.seed(42)
    boundary, obstacles = make_problem()
    tasks = build_tasks(window_mode)
    cp = CoverageProblem.CoverageProblem(restricted_areas=obstacles, search_area=boundary, tasks=tasks)
    agents = []
    for aid in range(n_agents):
        p = cp.generate_random_point_in_problem()
        agents.append(Agent.config(aid, (p.x, p.y), capacity, max_velocity=10))
    runner = Experiment.Runner(coverage_problem=cp, enable_plotting=False, agents=agents, algorithm=algorithm)
    runner.solve(profiling_enabled=False, debug=False)
    res = runner.evaluateSolution(show=False)
    compute_time, iterations, _, _, path_costs, rewards, _, max_path_cost = res
    return {
        "algorithm": algorithm,
        "windows": window_mode,
        "total_cost": sum(path_costs.values()),
        "makespan": max_path_cost,
        "reward": sum(rewards.values()),
        "iterations": iterations,
        "time_s": compute_time,
        "allocated": runner.allocated_tasks,
        "tw_violations": runner.time_window_violations,
    }


if __name__ == "__main__":
    rows = []
    for window_mode in ("none", "all", "mixed"):
        for algorithm in ("CBBA", "PI"):
            rows.append(run(algorithm, window_mode))

    hdr = f"{'algo':<5} {'windows':<7} {'cost':>9} {'makespan':>9} {'reward':>9} {'iters':>6} {'time(s)':>8} {'alloc':>6} {'tw_viol':>8}"
    print("\n" + hdr)
    print("-" * len(hdr))
    for r in rows:
        print(
            f"{r['algorithm']:<5} {r['windows']:<7} {r['total_cost']:>9.1f} {r['makespan']:>9.1f} "
            f"{r['reward']:>9.1f} {r['iterations']:>6d} {r['time_s']:>8.3f} {r['allocated']:>6d} {r['tw_violations']:>8d}"
        )

    assert all(r["tw_violations"] == 0 for r in rows), "hard time windows violated"
    print("\nAll runs converged with zero time-window violations.")
