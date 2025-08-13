#!/usr/bin/env python3
import math
from dataclasses import dataclass
from functools import cache
from multiprocessing import Pool
from typing import List

import numpy as np

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


def getTimeDiscountedReward(cost, Lambda, task: TrajectoryTask, agent_capacity):
    # return np.exp((Lambda - 1) * cost) * task.reward +1
    # return Lambda ** (cost) + task.reward
    norm_factor = 1 / agent_capacity
    tau = cost * norm_factor
    # result = Lambda ** (tau) * task.reward
    result = -math.log(tau) * task.reward
    # result = pow(Lambda, tau) * task.reward
    # This is not the fastest place to do the normalization, but in getTravelCost will break the time settings.
    return result


def getMinTravelCost(point, task: TrajectoryTask, environment):
    result = getTravelCost(point, task.start, environment)
    distance_to_end = getTravelCost(point, task.end, environment)
    shouldBeReversed = False
    if result > distance_to_end:
        result = distance_to_end
        shouldBeReversed = True
    return result, shouldBeReversed


def test_calculatePathRewardWithNewTask(environment, agent, taskCurr, taskPrev, timePrev, taskNext, timeNext, Lambda):
    # TODO
    # * Keep track of the time/distance of each task in the path and store it in a vector, just like the path
    # * Iterate through the vector and calculate

    if taskPrev == None:  # First task in the path
        dt, is_reversed = getMinTravelCost(agent.state, taskCurr, environment)
        minStart = max(taskCurr.start_time, agent.availability_time + dt)
    else:  # Not the first in the task
        dt, is_reversed = getMinTravelCost(taskPrev.end, taskCurr, environment)
        minStart = max(taskCurr.start_time, timePrev + distanceToCost(taskPrev.length) + dt)  # i have to have time to do task at j-1 and go to task m

    if taskNext == None:
        maxStart = taskCurr.end_time
    else:  # Not the last task in the path and we can still make the promised task
        dt, is_reversed = getMinTravelCost(taskCurr.end, taskNext, environment)
        maxStart = min(taskCurr.end_time, timeNext - distanceToCost(taskCurr.length) - dt)

    reward = getTimeDiscountedReward(dt, Lambda, taskCurr, 2000)
    penalty = getTravelCost(agent.state, taskCurr.start, environment)
    score = reward - penalty

    return score, minStart, maxStart


def calculatePathRewardWithNewTask(j, n, state, tasks, path, environment, Lambda, agent_capacity, use_single_point_estimation=False):
    temp_path = list(path)
    temp_path.insert(n, j)
    # print(j)
    is_reversed = False
    # travel cost to first task
    travel_cost = getTravelCost(state, tasks[temp_path[0]].start, environment)
    S_p = getTimeDiscountedReward(travel_cost, Lambda, tasks[temp_path[0]], agent_capacity)
    best_time = 0
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
                best_time = travel_cost
            elif p_idx == n:
                # the task after has to use the is_reversed bool to determine where to travel from
                if is_reversed:
                    travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
                else:
                    travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
            else:
                travel_cost += getTravelCost(previous_task.end, next_task.start, environment)
            # Scale the travelcost with the reward/priority
        S_p += getTimeDiscountedReward(travel_cost, Lambda, next_task, agent_capacity)

    # Add the cost for returning home
    travel_cost += getTravelCost(tasks[temp_path[-1]].end, state, environment)
    S_p += getTimeDiscountedReward(travel_cost, Lambda, tasks[temp_path[-1]], agent_capacity)
    return (S_p, is_reversed, best_time)


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
