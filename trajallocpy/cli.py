"""Command-line front-end: ``trajallocpy ...`` (see ``--help``).

Generates a random box scenario and runs the chosen allocation algorithm,
including the asynchronous ACBBA path with a configurable communication graph,
latency and packet loss.
"""

import argparse
import logging
import random

import numpy as np
import shapely

from trajallocpy import Agent, CoverageProblem, Experiment, Task, Transport


def _scenario(n_tasks, n_agents, capacity, seed, area=100.0):
    random.seed(seed)
    np.random.seed(seed)
    boundary = shapely.geometry.Polygon([[0, 0], [area, 0], [area, area], [0, area], [0, 0]])
    tasks = []
    for tid in range(n_tasks):
        x, y = random.uniform(0, area), random.uniform(0, area)
        seg = shapely.geometry.LineString([(x, y), (x + random.uniform(-5, 5), y + random.uniform(-5, 5))])
        tasks.append(Task.TrajectoryTask(tid, seg, reward=100))
    cp = CoverageProblem.CoverageProblem(restricted_areas=None, search_area=boundary, tasks=tasks)
    agents = [Agent.config(aid, (random.uniform(0, area), random.uniform(0, area)), capacity, max_velocity=10) for aid in range(n_agents)]
    return cp, agents


def _graph(kind, n):
    if kind == "ring":
        return Transport.CommunicationGraph.ring(n)
    if kind == "line":
        return Transport.CommunicationGraph.line(n)
    return Transport.CommunicationGraph.full(n)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="trajallocpy", description=__doc__)
    parser.add_argument("--algorithm", choices=["CBBA", "PI", "ACBBA"], default="CBBA")
    parser.add_argument("--agents", type=int, default=3)
    parser.add_argument("--tasks", type=int, default=8)
    parser.add_argument("--capacity", type=float, default=2000.0)
    parser.add_argument("--comm", choices=["full", "ring", "line"], default="full", help="ACBBA communication topology")
    parser.add_argument("--latency", type=float, default=0.0, help="ACBBA mean link latency (s)")
    parser.add_argument("--loss", type=float, default=0.0, help="ACBBA per-message drop probability")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--mode", choices=["step", "threads"], default="step", help="ACBBA execution mode")
    parser.add_argument("--max-runtime", type=float, default=30.0)
    parser.add_argument("--export-dir", default=None, help="write routes/tasks/transport JSON here")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING, format="%(message)s")

    cp, agents = _scenario(args.tasks, args.agents, args.capacity, args.seed)
    link = Transport.LinkModel(latency_mean=args.latency, loss_prob=args.loss)
    runner = Experiment.Runner(
        coverage_problem=cp,
        agents=agents,
        algorithm=args.algorithm,
        communication_graph=_graph(args.comm, args.agents),
        link_model=link,
        seed=args.seed,
        export_dir=args.export_dir,
        async_mode=args.mode,
        max_runtime=args.max_runtime,
    )
    runner.solve()
    result = runner.evaluateSolution(show=True)
    assigned = sorted(int(x) for robot in runner.robot_list.values() for x in robot.path)
    print(f"algorithm={args.algorithm} converged={runner.converged} iterations={runner.iterations}")
    print(f"allocated {result.allocated_tasks}/{args.tasks} tasks, max path cost {result.max_path_cost:.2f}")
    print(f"assignments: { {i: sorted(int(x) for x in r.path) for i, r in runner.robot_list.items()} }")
    if len(assigned) != len(set(assigned)):
        raise SystemExit("ERROR: allocation has conflicts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
