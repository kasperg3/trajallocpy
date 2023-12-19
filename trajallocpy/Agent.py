#!/usr/bin/env python3
import math
from dataclasses import dataclass
from functools import cache
from multiprocessing import Pool
from typing import List

from trajallocpy.Task import TrajectoryTask


@dataclass
class config:
    id: int
    position: list
    capacity: int  # time in seconds
    max_velocity: float = 3  # m/s
    max_acceleration: float = 1  # m/s^2


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


def distanceToCost(dist, max_velocity=3, max_acceleration=1):
    # Velocity ramp
    d_a = (max_velocity**2) / max_acceleration
    result = math.sqrt(4 * dist / max_acceleration) if dist < d_a else max_velocity / max_acceleration + dist / max_velocity
    return result


@cache
def getDistance(start, end, environment=None):
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
    if len(assigned_tasks) > 0:
        path, dist = environment.find_shortest_path(position, assigned_tasks[0].start, free_space_after=False, verify=False)
        full_path.extend(path)
        for i in range(len(assigned_tasks) - 1):
            full_path.extend(assigned_tasks[i].trajectory.coords)
            path, dist = environment.find_shortest_path(assigned_tasks[i].end, assigned_tasks[i + 1].start, free_space_after=False, verify=False)
            full_path.extend(path)
        full_path.extend(assigned_tasks[-1].trajectory.coords)
    return full_path


def getTravelCost(start, end, environment):
    return distanceToCost(getDistance(start, end, environment))


def getTimeDiscountedReward(cost, Lambda, task: TrajectoryTask):
    return Lambda ** (cost) * task.reward


def getMinTravelCost(point, task: TrajectoryTask, environment):
    result = getTravelCost(point, task.start, environment)
    distance_to_end = getTravelCost(point, task.end, environment)
    shouldBeReversed = False
    if result > distance_to_end:
        result = distance_to_end
        shouldBeReversed = True
    return result, shouldBeReversed


def calculatePathRewardWithNewTask(j, n, state, tasks, path, environment, use_single_point_estimation=False, Lambda=0.95):
    temp_path = list(path)
    temp_path.insert(n, j)
    # print(j)
    is_reversed = False
    # travel cost to first task
    travel_cost = getTravelCost(state, tasks[temp_path[0]].start, environment)
    S_p = getTimeDiscountedReward(travel_cost, Lambda, tasks[temp_path[0]])

    # Use a single point instead of greedily optimising the direction
    for p_idx in range(len(temp_path) - 1):
        previous_task = tasks[temp_path[p_idx]]
        next_task = tasks[temp_path[p_idx + 1]]
        if use_single_point_estimation:
            travel_cost += getTravelCost(previous_task.end, next_task.start)
        else:
            if p_idx == n - 1:
                # The task is inserted at n, when evaluating the task use n-1 to determine whether it should be reversed
                temp_cost, is_reversed = getMinTravelCost(previous_task.end, next_task, environment)
                travel_cost += temp_cost

            if p_idx == n:
                # the task after has to use the is_reversed bool to determine where to travel from
                if is_reversed:
                    travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
                else:
                    travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
            else:
                travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
            # Scale the travelcost with the reward/priority
        S_p += getTimeDiscountedReward(travel_cost, Lambda, next_task)

    # Add the cost for returning home
    travel_cost += getTravelCost(tasks[temp_path[-1]].end, state, environment)
    S_p += getTimeDiscountedReward(travel_cost, Lambda, tasks[temp_path[-1]])
    return S_p, is_reversed


def calculate_and_return(j, n, state, tasks, path, environment, use_single_point_estimation):
    S_pj, should_be_reversed = calculatePathRewardWithNewTask(j, n, state, tasks, path, environment, use_single_point_estimation)
    return j, n, S_pj, should_be_reversed


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
def calculatePathReward(position, task_list: List[TrajectoryTask], environment, Lambda=0.95):
    S_p = 0

    if len(task_list) > 0:
        travel_cost = getTravelCost(position, task_list[0].start, environment)
        S_p += getTimeDiscountedReward(travel_cost, Lambda, task_list[0])
        for t_index in range(len(task_list) - 1):
            travel_cost += getTravelCost(task_list[t_index].end, task_list[t_index + 1].start, environment)
            S_p += getTimeDiscountedReward(travel_cost, Lambda, task_list[t_index + 1])
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
