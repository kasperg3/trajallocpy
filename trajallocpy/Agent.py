#!/usr/bin/env python3
import math
from dataclasses import dataclass
from functools import cache
from typing import List, Optional, Tuple

from trajallocpy.Task import TrajectoryTask


@dataclass
class config:
    """Per-agent configuration passed to :class:`Experiment.Runner`.

    ``position`` accepts ``(x, y)``, ``[(x, y)]`` or a ``shapely`` point-like
    object; it is normalized to a plain ``(x, y)`` tuple. ``Lambda`` left as
    ``None`` means "use the algorithm's own default discount factor".
    """

    id: int
    position: object
    capacity: int  # time in seconds
    max_velocity: float = 3  # m/s
    max_acceleration: float = 1  # m/s^2
    Lambda: Optional[float] = None  # score discount; None -> algorithm default
    removal_threshold: int = 5

    def __post_init__(self):
        self.position = _normalize_position(self.position)


def _normalize_position(position) -> Tuple[float, float]:
    # shapely Point / geometry with .coords
    coords = getattr(position, "coords", None)
    if coords is not None:
        x, y = list(coords)[0][:2]
        return (float(x), float(y))
    seq = list(position)
    # [(x, y)] -> (x, y)
    if len(seq) == 1 and hasattr(seq[0], "__len__"):
        seq = list(seq[0])
    return (float(seq[0]), float(seq[1]))


# class Agent:


@dataclass
class BidInformation:
    y: float
    z: int
    t: float
    j: int
    k: int
    # winning_score: float
    # winning_agent: int
    # timestamp: float
    # task_id: int
    # sender_id: int


def distanceToCost(dist, max_velocity=5, max_acceleration=2):
    d_a = (max_velocity**2) / max_acceleration
    result = math.sqrt(4 * dist / max_acceleration) if dist < d_a else max_velocity / max_acceleration + dist / max_velocity
    return result


@cache
def getDistance(start, end, environment=None):
    # TODO this is a temporary fix for improving the performance of the code
    dist = [(a - b) ** 2 for a, b in zip(start, end)]
    dist = math.sqrt(sum(dist))
    return dist
    # If there is no environment defined, use euclidean
    if environment is None:
        # This is a optimised way of calculating euclidean distance: https://stackoverflow.com/questions/37794849/efficient-and-precise-calculation-of-the-euclidean-distance
        dist = [(a - b) ** 2 for a, b in zip(start, end)]
        dist = math.sqrt(sum(dist))
    else:
        path, dist = environment.find_shortest_path(start, end, free_space_after=False, verify=False)
    return dist


def getTravelPath(position, assigned_tasks, environment):
    full_path = []
    travel_paths = []
    task_paths = []
    if len(assigned_tasks) > 0:
        if environment is None:
            path = [position, assigned_tasks[0].start]
        else:
            path, dist = environment.find_shortest_path(position, assigned_tasks[0].start, free_space_after=False, verify=False)
        full_path.extend(path)
        for i in range(len(assigned_tasks) - 1):
            full_path.extend(assigned_tasks[i].trajectory.coords)

            if environment is None:
                path = [assigned_tasks[i].end, assigned_tasks[i + 1].start]
            else:
                path, dist = environment.find_shortest_path(assigned_tasks[i].end, assigned_tasks[i + 1].start, free_space_after=False, verify=False)
            full_path.extend(path)
            task_paths.append(assigned_tasks[i].trajectory.coords)
            travel_paths.append(path)
        full_path.extend(assigned_tasks[-1].trajectory.coords)
        task_paths.append(assigned_tasks[-1].trajectory.coords)

    return full_path, travel_paths, task_paths


@cache
def getTravelCost(start, end, environment):
    return distanceToCost(getDistance(start, end, environment))


_TAU_EPS = 1e-9


def getTimeDiscountedReward(cost, Lambda, task: TrajectoryTask, agent_capacity):
    # Bounded, monotonically non-increasing discount in (0, reward]. The old
    # -log(tau) form was unbounded and raised a domain error at cost==0.
    tau = cost / agent_capacity
    tau = min(max(tau, _TAU_EPS), 1.0)
    return (Lambda**tau) * task.reward


def _endpoints(task: TrajectoryTask, reverse: bool):
    if reverse:
        return task.end, task.start
    return task.start, task.end


def _has_deadline(task: TrajectoryTask) -> bool:
    # A task only has a hard time window when end_time is set (>0). Tasks left
    # at the default start_time==end_time==0 are treated as having no window.
    return task.end_time > 0


def getMinTravelCost(point, task: TrajectoryTask, environment):
    result = getTravelCost(point, task.start, environment)
    distance_to_end = getTravelCost(point, task.end, environment)
    shouldBeReversed = False
    if result > distance_to_end:
        result = distance_to_end
        shouldBeReversed = True
    return result, shouldBeReversed


def calculatePathRewardWithNewTask(j, n, state, tasks, path, environment, Lambda, agent_capacity, use_single_point_estimation=False):
    """Score of inserting task ``j`` at position ``n``.

    Returns ``(S_p, inserted_reversed, best_time, feasible)``. ``feasible`` is
    False when any task on the resulting path would have to start after its
    hard deadline (``end_time``); such an insertion must never be chosen.
    """
    temp_path = list(path)
    temp_path.insert(n, j)

    travel_cost = 0.0  # cumulative travel cost, drives the reward discount
    arrival = 0.0  # cumulative time incl. travel + execution + waiting (feasibility)
    feasible = True
    inserted_reversed = False
    best_time = 0.0
    prev_exit = state
    S_p = 0.0

    for t_idx in temp_path:
        task = tasks[t_idx]
        if use_single_point_estimation:
            reverse = False
            leg = getTravelCost(prev_exit, task.start, environment)
        else:
            cost_fwd = getTravelCost(prev_exit, task.start, environment)
            cost_rev = getTravelCost(prev_exit, task.end, environment)
            reverse = cost_rev < cost_fwd
            leg = cost_rev if reverse else cost_fwd
        _, exit_point = _endpoints(task, reverse)

        travel_cost += leg
        arrival += leg
        # Honor the earliest start time by waiting if we arrive early.
        if task.start_time > 0 and arrival < task.start_time:
            arrival = task.start_time
        # Hard time window: the task must begin no later than its deadline.
        if _has_deadline(task) and arrival > task.end_time:
            feasible = False
        arrival += distanceToCost(task.length)

        S_p += getTimeDiscountedReward(travel_cost, Lambda, task, agent_capacity)

        if t_idx == j:
            inserted_reversed = reverse
            best_time = travel_cost
        prev_exit = exit_point

    return (S_p, inserted_reversed, best_time, feasible)


# This is only used for evaluations!
def getTotalPathLength(position, task_list, environment):
    total_length = 0
    if len(task_list) != 0:
        # Add the cost of travelling to the first task
        total_length = getDistance(position, task_list[0].start, environment)
        # The cost of travelling between tasks
        for t_index in range(len(task_list) - 1):
            total_length += getDistance(task_list[t_index].end, task_list[t_index + 1].start, environment)
        # The cost of executing the task
        for t_index in range(len(task_list)):
            total_length += task_list[t_index].length
        # Add the cost of returning home
        total_length += getDistance(position, task_list[-1].end, environment)
    return total_length


def countTimeWindowViolations(position, task_list: List[TrajectoryTask], environment):
    """Number of tasks on the route that begin after their hard deadline."""
    violations = 0
    arrival = 0.0
    prev = position
    for task in task_list:
        arrival += getTravelCost(prev, task.start, environment)
        if task.start_time > 0 and arrival < task.start_time:
            arrival = task.start_time
        if _has_deadline(task) and arrival > task.end_time:
            violations += 1
        arrival += distanceToCost(task.length)
        prev = task.end
    return violations


def getArrivalTimes(position, task_list: List[TrajectoryTask], environment):
    """Cumulative travel cost to the start of each task in ``task_list``."""
    times = []
    cost = 0.0
    prev = position
    for task in task_list:
        cost += getTravelCost(prev, task.start, environment)
        times.append(cost)
        cost += distanceToCost(task.length)
        prev = task.end
    return times


def getTotalTaskLength(task_list):
    task_length = 0
    for t_index in range(len(task_list)):
        task_length += task_list[t_index].length
    return task_length


def getTotalTravelCost(position, task_list: List[TrajectoryTask], environment):
    total_cost = 0
    if len(task_list) != 0:
        # Add the cost of travelling to the first task
        total_cost = getTravelCost(position, task_list[0].start, environment)
        # The cost of travelling between tasks
        for t_index in range(len(task_list) - 1):
            total_cost += getTravelCost(task_list[t_index].end, task_list[t_index + 1].start, environment)
        # The cost of executing the task
        for t_index in range(len(task_list)):
            total_cost += distanceToCost(task_list[t_index].length)
        # Add the cost of returning home
        total_cost += getTravelCost(position, task_list[-1].end, environment)
    return total_cost


# S_i calculation of the agent
def calculatePathReward(position, task_list: List[TrajectoryTask], environment, agent_capacity, Lambda=0.95):
    S_p = 0

    if len(task_list) > 0:
        travel_cost = getTravelCost(position, task_list[0].start, environment)
        S_p += getTimeDiscountedReward(travel_cost, Lambda, task_list[0], agent_capacity)
        for t_index in range(len(task_list) - 1):
            travel_cost += getTravelCost(task_list[t_index].end, task_list[t_index + 1].start, environment)
            S_p += getTimeDiscountedReward(travel_cost, Lambda, task_list[t_index + 1], agent_capacity)
    return S_p


def getTrajectory(task_list: List[TrajectoryTask]):
    trajectory = []
    if len(task_list) > 0:
        trajectory.append(task_list[0].start)
        for t_index in range(len(task_list) - 1):
            trajectory.extend(task_list[t_index].trajectory.coords)
            trajectory.append(task_list[t_index].end)
        trajectory.extend(task_list[-1].trajectory.coords)
        trajectory.append(task_list[-1].end)
    return trajectory
