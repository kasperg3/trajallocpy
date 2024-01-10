import copy
import math
import random
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import dataclass
from functools import cache
from multiprocessing import Pool
from typing import List

import numpy as np

from trajallocpy.Agent import *
from trajallocpy.Task import TrajectoryTask


class agent:
    def __init__(
        self,
        state,
        id,
        capacity=0,
        environment=None,
        tasks=None,
        color=None,
        point_estimation=False,
        max_velocity=3,
        max_acceleration=1,
    ):
        self.environment = environment
        self.tasks = None
        if tasks is not None:
            self.tasks = {x.id: x for x in copy.deepcopy(tasks)}

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
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        # Agent ID
        self.id = id

        # Local Winning Agent List
        self.z = {}
        # Local Winning Bid List
        self.y = {}
        # Time Stamp List
        self.t = {}
        # Bundle
        self.bundle = []
        # Path
        self.path = []
        # Maximum task capacity
        if capacity is None:
            raise Exception("Error: agent capacity cannot be None")
        else:
            self.capacity = capacity

        # initialize state
        if state is None:
            raise Exception("ERROR: Initial state cannot be None")
        else:
            self.state = state.coords[0]
        # socre function parameters
        self.Lambda = 0.95

        self.removal_list = {}
        self.removal_threshold = 5  # TODO find a good value for this when ros is implemented
        self.message_history = []

    def __repr__(self) -> str:
        return f"Agent {self.id} \n path {self.path} \n  bundle {self.bundle} \n y(winning bids) {self.y} \n z(winning agents) {self.z} \n t(timestamps) {self.t} \n"

    def add_tasks(self, tasks):
        # add the tasks to self.tasks dictionary
        for task in tasks:
            self.tasks[task.id] = task

    def __str__(self) -> str:
        return f"Agent {self.id} \n path {self.path} \n  bundle {self.bundle} \n y(winning bids) {self.y} \n z(winning agents) {self.z} \n t(timestamps) {self.t} \n"

    def getPathTasks(self) -> List[TrajectoryTask]:
        result = []
        for task in self.path:
            result.append(self.tasks.get(task))
        return result

    def send_message(self):  # TODO rename and make it return bidinformation
        return self.y, self.z, self.t

    def getCij(self):
        # Calculate Sp_i
        S_p = calculatePathReward(self.state, self.getPathTasks(), self.environment, self.Lambda)
        # init
        best_pos = None
        c = 0
        reverse = None
        best_task = None
        # try all tasks
        for j, task in self.tasks.items():
            # If already in the bundle list
            if j in self.bundle or self.removal_list.get(j, 0) > self.removal_threshold:
                continue  # Do not include if already in the bundle or if the removal threshold is exceeded
            else:
                for n in range(len(self.path) + 1):
                    S_pj, should_be_reversed = calculatePathRewardWithNewTask(
                        j, n, self.state, self.tasks, self.path, self.environment, self.use_single_point_estimation
                    )
                    c_ijn = S_pj - S_p

                    if c_ijn > c and c_ijn > self.y.get(j, -1):  # TODO: Figure out if this is bad for the general solution
                        c = c_ijn  # Store the cost
                        best_pos = n
                        reverse = should_be_reversed
                        best_task = j

        # reverse the task with max reward if necesarry
        if reverse:
            self.tasks[j].reverse()

        return best_task, best_pos, c

    # TODO this might be worth looking more into.
    # def getCij(self):
    #     # Calculate Sp_i
    #     S_p = calculatePathReward(self.state, self.getPathTasks(), self.environment, self.Lambda)
    #     # init
    #     best_pos = None
    #     c = 0
    #     reverse = None
    #     best_task = None
    #     executor_tasks = []

    #     for j, task in self.tasks.items():
    #         if j in self.bundle or self.removal_list.get(j, 0) > self.removal_threshold:
    #             continue  # Do not include if already in the bundle or if the removal threshold is exceeded
    #         else:
    #             for n in range(len(self.path) + 1):
    #                 executor_tasks.append((j, n))

    #     for test in executor_tasks:
    #         j, n = test
    #         S_pj, should_be_reversed = calculatePathRewardWithNewTask(j, n, self.state, self.tasks, self.path, self.environment)
    #         c_ijn = S_pj - S_p
    #         if c_ijn > c and c_ijn > self.y.get(j, -1):
    #             c = c_ijn
    #             best_pos = n
    #             reverse = should_be_reversed
    #             best_task = j

    #     # reverse the task with max reward if necesarry
    #     if reverse:
    #         self.tasks[j].reverse()

    #     return best_task, best_pos, c

    ###
    # with ThreadPoolExecutor() as executor:
    #     executor_tasks = []
    #     for j, task in self.tasks.items():
    #         if j in self.bundle or self.removal_list.get(j, 0) > self.removal_threshold:
    #             continue  # Do not include if already in the bundle or if the removal threshold is exceeded
    #         else:
    #             for n in range(len(self.path) + 1):
    #                 executor_tasks.append((j, n, self.state, self.tasks, self.path, None, self.use_single_point_estimation))
    #     for j, n, S_pj, should_be_reversed in executor.map(calculate_and_return, *zip(*executor_tasks)):
    #         c_ijn = S_pj - S_p
    #         if c_ijn > c and c_ijn > self.y.get(j, -1):
    #             c = c_ijn
    #             best_pos = n
    #             reverse = should_be_reversed
    #             best_task = j
    #         # reverse the task with max reward if necessary
    # if reverse:
    #     self.tasks[best_task].reverse()
    # return best_task, best_pos, c

    def build_bundle(self):
        if self.tasks is None:
            return
        bid_list = []
        bundle_time = time.monotonic()
        while getTotalTravelCost(self.state, self.getPathTasks(), self.environment) <= self.capacity:
            J_i, n_J, c = self.getCij()
            if J_i is None:
                break
            self.bundle.append(J_i)
            self.path.insert(n_J, J_i)

            self.y[J_i] = c
            self.z[J_i] = self.id
            self.t[J_i] = bundle_time  # Update the time of the winning bet
            bid_list.append(BidInformation(y=c, z=self.id, t=bundle_time, j=J_i, k=self.id))
        return bid_list

    def __update_time(self, task):
        self.t[task] = time.monotonic()

    def __action_rule(self, k, j, task, z_kj, y_kj, t_kj, z_ij, y_ij, t_ij) -> BidInformation:
        eps = 20
        i = self.id
        sender_info = BidInformation(y=y_kj, z=z_kj, t=t_kj, j=j, k=self.id)
        own_info = BidInformation(y=y_ij, z=z_ij, t=t_ij, j=j, k=self.id)
        if z_kj == k:  # Rule 1 Agent k thinks k is z_kj
            if z_ij == i:  # Rule 1.1
                if y_kj > y_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info
                elif y_kj == y_ij and z_kj < z_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info
                elif y_kj < y_ij:
                    self.__update_time(task)
                    return own_info

            elif z_ij == k:  # Rule 1.2
                if t_kj > t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return None
                elif abs(t_kj - t_ij) < eps:
                    self.__leave()
                    return None
                elif t_kj < t_ij:
                    self.__leave()
                    return None

            elif z_ij != i and z_ij != k:  # Rule 1.3
                if y_kj > y_ij and t_kj >= t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info

                elif y_kj < y_ij and t_kj <= t_ij:
                    self.__leave()
                    return own_info

                elif y_kj == y_ij:
                    self.__leave()
                    return own_info

                elif y_kj < y_ij and t_kj > t_ij:
                    self.__reset(task)
                    return sender_info

                elif y_kj > y_ij and t_kj < t_ij:
                    self.__reset(task)
                    return sender_info

            elif z_ij == -1:  # Rule 1.4
                self.__update(y_kj, z_kj, t_kj, task)
                return sender_info

        elif z_kj == i:  # Rule 2 Agent k thinks winning agent is i
            if z_ij == i and (abs(t_kj - t_ij) < eps):  # Rule 2.1 # Agent i thinks itself is the winner
                self.__leave()
                return None

            elif z_ij == k:
                self.__reset(task)
                return sender_info

            elif z_ij != i and z_ij != k:
                self.__leave()
                return own_info

            elif z_ij == -1:
                self.__leave()
                return own_info

        elif z_kj != k and z_kj != i:  # Rule 3 Agent k think the winner of task j is not the itself nor agent i
            if z_ij == i:  # Rule 3.1
                if y_kj > y_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info

                elif y_kj == y_ij and z_kj < z_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info

                elif y_kj < y_ij:
                    self.__update_time(task)
                    return own_info

            elif z_ij == k:  # Rule 3.2
                if t_kj >= t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info
                elif t_kj < t_ij:
                    self.__reset(task)
                    return sender_info

            elif z_kj == z_ij:  # Rule 3.3
                if t_kj > t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return None
                elif abs(t_kj - t_ij) <= eps:
                    self.__leave()
                    return None
                elif t_kj < t_ij:
                    self.__leave()
                    return None

            elif z_ij != i and z_ij != k:  # Rule 3.4
                if y_kj > y_ij and t_kj >= t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info
                elif y_kj < y_ij and t_kj <= t_ij:
                    self.__leave()
                    return own_info
                elif y_kj == y_ij:
                    self.__leave()
                    return own_info
                elif y_kj < y_ij and t_kj > t_ij:
                    self.__reset(task)
                    return sender_info
                elif y_kj > y_ij and t_kj < t_ij:
                    self.__reset(task)
                    return sender_info

            elif z_ij == -1:  # Rule 3.5
                self.__update(y_kj, z_kj, t_kj, task)
                return sender_info

        elif z_kj == -1:  # Rule 4 Agent k thinks None is z_kj
            if z_ij == i:
                self.__leave()
                return own_info
            elif z_ij == k:
                self.__update(y_kj, z_kj, t_kj, task)
                return sender_info
            elif z_ij != i and z_ij != k:
                if t_kj > t_ij:
                    self.__update(y_kj, z_kj, t_kj, task)
                    return sender_info
            elif z_ij == -1:
                self.__leave()
                return None
        # Default leave and rebroadcast own info
        self.__leave()
        return own_info

    def __rebroadcast(self, information):
        y = information["y"]
        z = information["z"]
        t = information["t"]
        self.send_information(y, z, t, self.id)

    def __receive_information(self):
        raise NotImplementedError()
        # message = self.my_socket.recieve(self.agent)
        # if message is None:
        #     return None
        # return message

    def send_information(self, y, z, t, k):
        """This function is used for sharing information between agents and is not implemented in this base class
        Raises
        ------
        NotImplementedError
            _description_
        """
        raise NotImplementedError()
        # msg = {self.agent: {"y": y, "z": z, "t": t}}
        # self.my_socket.send(self.agent, msg, k)

    def update_task_async(self, bids: List[BidInformation]):
        # Update Process
        rebroadcasts = []
        for bid_info in bids:
            j = bid_info.j
            k = bid_info.k

            # Own info
            y_ij = self.y.get(j, 0)
            z_ij = self.z.get(j, -1)
            t_ij = self.t.get(j, 0)

            # Recieved info
            y_kj = bid_info.y  # Winning bids
            z_kj = bid_info.z  # Winning agent
            t_kj = bid_info.t  # Timestamps

            rebroadcast = self.__action_rule(k=k, j=j, task=j, z_kj=z_kj, y_kj=y_kj, t_kj=t_kj, z_ij=z_ij, y_ij=y_ij, t_ij=t_ij)
            if rebroadcast is not None:
                rebroadcasts.append(rebroadcast)
        return rebroadcasts

    def update_task(self, Y: List[BidInformation]):
        # Update Process
        rebroadcasts = []

        for k in Y:
            for j in self.tasks:
                # Recieve info
                y_kj = Y[k][0].get(j, 0)  # Winning bids
                z_kj = Y[k][1].get(j, -1)  # Winning agent
                t_kj = Y[k][2].get(j, 0)  # Timestamps

                # Own info
                y_ij = self.y.get(j, 0)
                z_ij = self.z.get(j, -1)
                t_ij = self.t.get(j, 0)
                # TODO parse the information in a better way
                rebroadcast = self.__action_rule(k=k, j=j, task=j, z_kj=z_kj, y_kj=y_kj, t_kj=t_kj, z_ij=z_ij, y_ij=y_ij, t_ij=t_ij)
                if rebroadcast:
                    # TODO save the rebroadcasts
                    rebroadcasts.append(rebroadcast)
        return rebroadcasts

    def __update(self, y_kj, z_kj, t_kj, j):
        """
        Update values
        """
        self.y[j] = y_kj
        self.z[j] = z_kj
        self.t[j] = t_kj
        self.__update_path(j)

    def __update_path(self, task):
        if task not in self.bundle:
            return
        index = self.bundle.index(task)
        b_retry = self.bundle[index + 1 :]
        for idx in b_retry:
            self.y[idx] = 0
            self.z[idx] = -1
            self.t[idx] = time.monotonic()

        self.removal_list[task] = self.removal_list.get(task, 0) + 1
        self.path = [num for num in self.path if num not in self.bundle[index:]]
        self.bundle = self.bundle[:index]

    def __reset(self, task):
        self.y[task] = 0
        self.z[task] = -1
        self.t[task] = time.monotonic()
        self.__update_path(task)

    def __leave(self):
        """
        Do nothing
        """
        return
