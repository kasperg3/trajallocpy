import copy
import itertools
import multiprocessing
import random
from typing import List

import numpy as np

from trajallocpy import Agent
from trajallocpy.Task import TrajectoryTask

EPSILON = np.finfo(float).eps


class BundleResult:
    def __init__(self, agent: Agent):
        self.bundle = agent.bundle
        self.path = agent.path
        self.winning_agents = agent.winning_agents
        self.winning_bids = agent.winning_bids
        self.id = agent.id


class agent:
    def __init__(
        self,
        state,
        id,
        environment=None,
        number_of_agents=None,
        capacity=None,
        tasks=None,
        color=None,
        point_estimation=False,
    ):
        self.environment = environment
        self.tasks = copy.deepcopy(tasks)
        self.task_num = len(tasks)
        self.use_single_point_estimation = point_estimation
        if color is None:
            self.color = (
                random.uniform(0, 1),
                random.uniform(0, 1),
                random.uniform(0, 1),
            )
        else:
            self.color = color

        # TODO this should be configurable
        self.max_velocity = 3
        self.max_acceleration = 1

        # Agent ID
        self.id = id

        # Local Winning Agent List
        self.winning_agents = np.ones(self.task_num, dtype=np.int8) * self.id
        # Local Winning Bid List
        self.winning_bids = np.array([0 for _ in range(self.task_num)], dtype=np.float64)
        # Bundle
        self.bundle = []
        # Path
        self.path = []
        # times: List of time in seconds to each task in the path
        self.times = []
        # Maximum task capacity
        if capacity is None:
            raise Exception("Error: agent capacity cannot be None")
        else:
            self.capacity = capacity

        # Local Clock
        self.time_step = 0
        # Time Stamp List
        self.timestamps = {a: self.time_step for a in range(number_of_agents)}

        # initialize state
        if state is None:
            raise Exception("ERROR: Initial state cannot be None")
        else:
            self.state = state.coords[0]
        # socre function parameters
        self.Lambda = 0.99

        self.availability_time = 0

        self.removal_list = np.zeros(self.task_num, dtype=np.int8)
        self.removal_threshold = 5

    def update_bundle_result(self, state: BundleResult):
        if self.id == state.id:
            self.bundle = state.bundle
            self.path = state.path
            self.winning_agents = state.winning_agents
            self.winning_bids = state.winning_bids

    def add_tasks(self, tasks):
        self.tasks.extend(tasks)

    def getPathTasks(self) -> List[TrajectoryTask]:
        return self.tasks[self.path]

    def send_message(self):
        return self.winning_bids.tolist(), self.winning_agents.tolist(), self.timestamps

    def receive_message(self, Y):
        self.Y = Y

    def getCij(self):
        """
        Returns the cost list c_ij for agent i where the position n results in the greatest reward
        """
        # Calculate Sp_i
        S_p = Agent.calculatePathReward(self.state, self.getPathTasks(), self.environment, self.Lambda)
        # init
        best_pos = np.zeros(self.task_num, dtype=int)
        c = np.zeros(self.task_num)
        reverse = np.zeros(self.task_num)
        best_time = 0
        # Collect the tasks which should be considered for planning
        ignore_tasks = [key for key, value in enumerate(self.removal_list) if value > self.removal_threshold]
        tasks_to_check = set(range(len(self.tasks))).difference(self.bundle).difference(ignore_tasks)

        for n, j in itertools.product(range(len(self.path) + 1), tasks_to_check):
            S_pj, should_be_reversed, best_time = Agent.calculatePathRewardWithNewTask(
                j, n, self.state, self.tasks, self.path, self.environment, self.Lambda, self.use_single_point_estimation
            )
            c_ijn = S_pj - S_p
            if c[j] < c_ijn:
                c[j] = c_ijn  # Store the cost
                best_pos[j] = n
                reverse[j] = should_be_reversed

        return (best_pos, c, reverse, best_time)

    def update_time(self, index, time):
        self.times.insert(index, time)
        # Correct the times after the insertion
        for i in range(index + 1, len(self.times)):
            self.times[i] += time

    def build_bundle(self, queue: multiprocessing.Queue):
        while Agent.getTotalTravelCost(self.state, self.getPathTasks(), self.environment) <= self.capacity:
            best_pos, c, reverse, best_time = self.getCij()
            D1 = c - self.winning_bids > EPSILON
            D2 = abs(c - self.winning_bids) <= EPSILON
            h = D1 | (D2 & (self.id < self.winning_agents))
            if sum(h) == 0:  # No valid task
                break

            c[~h] = 0
            J_i = np.argmax(c)
            n_J = best_pos[J_i]

            # reverse the task with max reward if necesarry
            if reverse[J_i]:
                self.tasks[J_i].reverse()

            self.bundle.append(J_i)
            self.path.insert(n_J, J_i)
            self.update_time(n_J, best_time)

            self.winning_bids[J_i] = c[J_i]
            self.winning_agents[J_i] = self.id

        queue.put(BundleResult(self))

    def update_task(self):
        id_list = list(self.Y.keys())
        id_list.insert(0, self.id)

        # Update time list
        for id in list(self.timestamps.keys()):
            if id in id_list:
                self.timestamps[id] = self.time_step
            else:
                s_list = []
                for neighbor_id in id_list[1:]:
                    s_list.append(self.Y[neighbor_id][2][id])
                if len(s_list) > 0:
                    self.timestamps[id] = max(s_list)

        # Update Process
        for j in range(self.task_num):
            for k in id_list[1:]:
                y_k = self.Y[k][0]
                z_k = self.Y[k][1]
                s_k = self.Y[k][2]
                i = self.id

                z_ij = self.winning_agents[j]
                z_kj = z_k[j]
                y_kj = y_k[j]
                y_ij = self.winning_bids[j]
                # Rule Based Update
                # Rule 1~4
                if z_kj == k:
                    # Rule 1
                    if z_ij == self.id:
                        if y_kj > y_ij:
                            self.__update(j, y_kj, z_kj)
                        elif abs(y_kj - y_ij) < EPSILON:  # Tie Breaker
                            if k < self.id:
                                self.__update(j, y_kj, z_kj)
                        else:
                            self.__leave()
                    # Rule 2
                    elif z_ij == k:
                        self.__update(j, y_kj, z_kj)
                    # Rule 3
                    elif z_ij != -1:
                        m = z_ij
                        if (
                            (s_k[m] > self.timestamps[m]) or (y_kj > y_ij) or (abs(y_kj - y_ij) < EPSILON and k < self.id)
                        ):  # Combine conditions using logical or
                            self.__update(j, y_kj, z_kj)
                    # Rule 4
                    elif z_ij == -1:
                        self.__update(j, y_kj, z_kj)
                    else:
                        raise Exception("Error while updating")
                # Rule 5~8
                elif z_kj == i:
                    # Rule 5
                    if z_ij == i:
                        self.__leave()
                    # Rule 6
                    elif z_ij == k:
                        self.__reset(j)
                    # Rule 7
                    elif z_ij != -1:
                        m = z_ij
                        if s_k[m] > self.timestamps[m]:
                            self.__reset(j)
                    # Rule 8
                    elif z_ij == -1:
                        self.__leave()
                    else:
                        raise Exception("Error while updating")
                # Rule 9~13
                elif z_kj != -1:
                    m = z_kj
                    # Rule 9
                    if z_ij == i:
                        if (s_k[m] >= self.timestamps[m]) and ((y_kj > y_ij) or (abs(y_kj - y_ij) < EPSILON and m < self.id)):
                            self.__update(j, y_kj, z_kj)
                    # Rule 10
                    elif z_ij == k:
                        if s_k[m] > self.timestamps[m]:
                            self.__update(j, y_kj, z_kj)
                        else:
                            self.__reset(j)
                    # Rule 11
                    elif z_ij == m:
                        if s_k[m] > self.timestamps[m]:
                            self.__update(j, y_kj, z_kj)
                    # Rule 12
                    elif z_ij != -1:
                        n = z_ij
                        if (
                            (s_k[m] > self.timestamps[m])
                            and (s_k[n] > self.timestamps[n])
                            or (s_k[m] > self.timestamps[m])
                            and (y_kj > y_ij)
                            or (s_k[m] > self.timestamps[m])
                            and (abs(y_kj - y_ij) < EPSILON and m < n)
                            or (s_k[n] > self.timestamps[n])
                            and (self.timestamps[m] > s_k[m])
                        ):
                            self.__update(j, y_kj, z_kj)
                    # Rule 13
                    elif z_ij == -1:
                        if s_k[m] > self.timestamps[m]:
                            self.__update(j, y_kj, z_kj)
                    else:
                        raise Exception("Error while updating")
                # Rule 14~17
                elif z_kj == -1:
                    # Rule 14
                    if z_ij == i:
                        self.__leave()
                    # Rule 15
                    elif z_ij == k:
                        self.__update(j, y_kj, z_kj)
                    # Rule 16
                    elif z_ij != -1:
                        m = z_ij
                        if s_k[m] > self.timestamps[m]:
                            self.__update(j, y_kj, z_kj)
                    # Rule 17
                    elif z_ij == -1:
                        self.__leave()
                    else:
                        raise Exception("Error while updating")
                else:
                    raise Exception("Error while updating")

        self.time_step += 1

        converged = False
        return converged

    def __update_path(self, task):
        if task not in self.bundle:
            return
        index = self.bundle.index(task)
        b_retry = self.bundle[index + 1 :]
        for idx in b_retry:
            self.winning_bids[idx] = 0
            self.winning_agents[idx] = -1

        self.removal_list[task] = self.removal_list[task] + 1
        self.path = [num for num in self.path if num not in self.bundle[index:]]
        self.bundle = self.bundle[:index]

    def __update(self, j, y_kj, z_kj):
        """
        Update values
        """
        self.winning_bids[j] = y_kj
        self.winning_agents[j] = z_kj
        self.__update_path(j)

    def __reset(self, j):
        """
        Reset values
        """
        self.winning_bids[j] = 0
        self.winning_agents[j] = -1  # -1 means "none"
        self.__update_path(j)

    def __leave(self):
        """
        Do nothing
        """
        pass
