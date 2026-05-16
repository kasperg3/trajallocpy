import math

import numpy as np
import pytest
import shapely

from trajallocpy import Agent, CoverageProblem, Experiment, Task


def test_dummy():
    assert 1 == 1


def test_import():
    import trajallocpy  # noqa: F401


# --- Task.reverse correctness -------------------------------------------------


def test_reverse_is_involution_and_consistent():
    task = Task.TrajectoryTask(0, shapely.geometry.LineString([(0, 0), (1, 2), (3, 4)]))
    original = list(task.trajectory.coords)

    task.reverse()
    # start/end must match the (reversed) coordinate order
    assert task.start == task.trajectory.coords[0]
    assert task.end == task.trajectory.coords[-1]
    assert list(task.trajectory.coords) == original[::-1]

    task.reverse()
    assert list(task.trajectory.coords) == original
    assert task.start == original[0]
    assert task.end == original[-1]


# --- Time-discounted reward ---------------------------------------------------


def test_time_discounted_reward_is_bounded_and_monotone():
    task = Task.TrajectoryTask(0, shapely.geometry.LineString([(0, 0), (1, 0)]), reward=100)
    r0 = Agent.getTimeDiscountedReward(0.0, 0.99, task, 1000)  # no domain error
    r_mid = Agent.getTimeDiscountedReward(500.0, 0.99, task, 1000)
    r_far = Agent.getTimeDiscountedReward(5000.0, 0.99, task, 1000)
    assert math.isfinite(r0)
    assert 0 < r_far <= r_mid <= r0 <= task.reward


# --- Problem helpers ----------------------------------------------------------


def _make_problem(window_mode, n_agents=3, capacity=2000, seed=42):
    np.random.seed(seed)
    boundary = shapely.geometry.Polygon([[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]])
    obstacles = shapely.geometry.MultiPolygon(
        [shapely.geometry.Polygon([[20, 20], [40, 20], [40, 40], [20, 40], [20, 20]])]
    )
    segments = [
        [(10, 10), (15, 15)],
        [(50, 30), (70, 35)],
        [(30, 70), (45, 85)],
        [(85, 15), (90, 25)],
        [(20, 55), (35, 60)],
    ]
    tasks = []
    for tid, seg in enumerate(segments):
        windowed = window_mode == "all" or (window_mode == "mixed" and tid % 2 == 0)
        tasks.append(Task.TrajectoryTask(tid, shapely.geometry.LineString(seg), reward=100, end_time=500.0 if windowed else 0.0))
    cp = CoverageProblem.CoverageProblem(restricted_areas=obstacles, search_area=boundary, tasks=tasks)
    agents = []
    for aid in range(n_agents):
        p = cp.generate_random_point_in_problem()
        agents.append(Agent.config(aid, (p.x, p.y), capacity, max_velocity=10))
    return cp, agents, len(tasks)


def _run(algorithm, window_mode, **kw):
    cp, agents, n_tasks = _make_problem(window_mode, **kw)
    runner = Experiment.Runner(coverage_problem=cp, enable_plotting=False, agents=agents, algorithm=algorithm)
    runner.solve(profiling_enabled=False, debug=False)
    runner.evaluateSolution(show=False)
    return runner, n_tasks


def _assert_conflict_free(runner):
    assigned = []
    for robot in runner.robot_list.values():
        assigned.extend(robot.path)
    assert len(assigned) == len(set(assigned)), f"a task was assigned to more than one agent: {assigned}"


# --- CBBA + PI across all time-window regimes --------------------------------


@pytest.mark.parametrize("algorithm", ["CBBA", "PI"])
@pytest.mark.parametrize("window_mode", ["none", "all", "mixed"])
def test_allocation_valid(algorithm, window_mode):
    runner, n_tasks = _run(algorithm, window_mode)
    _assert_conflict_free(runner)
    # Converged (the solve loop only exits on conflicts==0 / single agent).
    assert isinstance(runner.iterations, int)
    # Hard time windows are respected.
    assert runner.time_window_violations == 0
    # The allocation makes progress on a feasible instance.
    assert runner.allocated_tasks > 0
    assert runner.allocated_tasks <= n_tasks


def test_cbba_bids_are_diminishing_within_bundle():
    runner, _ = _run("CBBA", "mixed")
    for robot in runner.robot_list.values():
        bids = [robot.winning_bids[t] for t in robot.bundle]
        for earlier, later in zip(bids, bids[1:]):
            assert later <= earlier + 1e-6, f"DMG violated in bundle bids: {bids}"


def test_unreachable_windowed_task_is_left_unassigned():
    np.random.seed(7)
    boundary = shapely.geometry.Polygon([[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]])
    reachable = Task.TrajectoryTask(0, shapely.geometry.LineString([(48, 48), (52, 52)]), reward=100)
    # Far away with an impossibly tight deadline -> can never start in time.
    unreachable = Task.TrajectoryTask(1, shapely.geometry.LineString([(95, 95), (99, 99)]), reward=100, end_time=0.001)
    cp = CoverageProblem.CoverageProblem(restricted_areas=None, search_area=boundary, tasks=[reachable, unreachable])
    agents = [Agent.config(0, (1.0, 1.0), 2000, max_velocity=10)]
    runner = Experiment.Runner(coverage_problem=cp, enable_plotting=False, agents=agents, algorithm="CBBA")
    runner.solve()
    runner.evaluateSolution(show=False)
    assigned = []
    for robot in runner.robot_list.values():
        assigned.extend(robot.path)
    assert 1 not in assigned
    assert runner.time_window_violations == 0


if __name__ == "__main__":
    pytest.main(["-v", "-x", "tests/allocation_test.py"])
