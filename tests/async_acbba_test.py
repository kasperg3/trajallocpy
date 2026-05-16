"""Decentralization properties of the asynchronous ACBBA path.

These prove conflict-freedom and convergence without a global barrier under
partial connectivity, latency and packet loss, that runs are deterministic
given a seed, that termination is detected by quiescence (not the timeout),
and that hard time windows are still respected.
"""

import random
import sys
from pathlib import Path

import numpy as np
import pytest
import shapely

from trajallocpy import Agent, CoverageProblem, Experiment, Task, Transport

sys.path.insert(0, str(Path(__file__).parent))
from allocation_test import _make_problem  # noqa: E402


def _seeded_problem(window_mode="none"):
    random.seed(1)
    np.random.seed(42)
    return _make_problem(window_mode)


def _assigned(runner):
    out = []
    for robot in runner.robot_list.values():
        out.extend(int(x) for x in robot.path)
    return out


def _assert_conflict_free(runner):
    assigned = _assigned(runner)
    assert len(assigned) == len(set(assigned)), f"task assigned to more than one agent: {assigned}"


def _solve(*, graph=None, link=None, seed=11, window="none", mode="step", max_runtime=60.0):
    cp, agents, n_tasks = _seeded_problem(window)
    runner = Experiment.Runner(
        coverage_problem=cp,
        agents=agents,
        algorithm="ACBBA",
        communication_graph=graph,
        link_model=link,
        seed=seed,
        async_mode=mode,
        max_runtime=max_runtime,
    )
    runner.solve()
    runner.evaluateSolution(show=False)
    return runner, n_tasks


def test_acbba_conflict_free_full_mesh_no_loss():
    runner, n = _solve()
    _assert_conflict_free(runner)
    assert runner.converged is True
    assert runner.time_window_violations == 0
    assert 0 < runner.allocated_tasks <= n


def test_acbba_partial_connectivity():
    cp, agents, _ = _seeded_problem()
    graph = Transport.CommunicationGraph.ring(len(agents))
    runner, _ = _solve(graph=graph)
    _assert_conflict_free(runner)
    assert runner.converged is True


def test_acbba_latency_and_loss():
    link = Transport.LinkModel(latency_mean=0.05, latency_jitter=0.02, loss_prob=0.15)
    runner, _ = _solve(link=link, max_runtime=120.0)
    _assert_conflict_free(runner)
    assert runner.converged is True
    assert runner.transport_layer.dropped > 0  # loss was actually exercised


def test_acbba_deterministic_with_seed():
    def once():
        runner, _ = _solve(seed=99)
        return ({i: sorted(int(x) for x in r.path) for i, r in runner.robot_list.items()}, runner.iterations)

    assert once() == once()


def test_acbba_termination_detected_without_barrier():
    runner, _ = _solve(max_runtime=60.0)
    assert runner.converged is True
    # Quiescence, not the timeout: nothing in flight, all messages accounted.
    assert runner.transport_layer.in_flight() == 0
    assert runner.transport_layer.sent == runner.transport_layer.received


def test_acbba_respects_hard_time_windows():
    np.random.seed(7)
    boundary = shapely.geometry.Polygon([[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]])
    reachable = Task.TrajectoryTask(0, shapely.geometry.LineString([(48, 48), (52, 52)]), reward=100)
    unreachable = Task.TrajectoryTask(1, shapely.geometry.LineString([(95, 95), (99, 99)]), reward=100, end_time=0.001)
    cp = CoverageProblem.CoverageProblem(restricted_areas=None, search_area=boundary, tasks=[reachable, unreachable])
    agents = [Agent.config(0, (1.0, 1.0), 2000, max_velocity=10)]
    runner = Experiment.Runner(coverage_problem=cp, agents=agents, algorithm="ACBBA", seed=1)
    runner.solve()
    runner.evaluateSolution(show=False)
    assert 1 not in _assigned(runner)
    assert runner.time_window_violations == 0


def test_acbba_dynamic_graph_self_heals():
    cp, agents, _ = _seeded_problem()
    n = len(agents)

    def schedule(t):
        adjacency = np.ones((n, n)) - np.eye(n)
        if t < 0.2:  # isolate the 0-1 link early, then restore full mesh
            adjacency[0, 1] = 0
            adjacency[1, 0] = 0
        return adjacency

    graph = Transport.CommunicationGraph(n).with_schedule(schedule)
    runner, _ = _solve(graph=graph)
    _assert_conflict_free(runner)
    assert runner.converged is True


def test_acbba_threads_mode():
    runner, _ = _solve(mode="threads", max_runtime=15.0)
    _assert_conflict_free(runner)
    assert runner.converged is True


@pytest.mark.parametrize("window_mode", ["none", "all", "mixed"])
def test_acbba_all_window_modes(window_mode):
    runner, n = _solve(window=window_mode)
    _assert_conflict_free(runner)
    assert runner.time_window_violations == 0
    assert 0 < runner.allocated_tasks <= n
