import copy
import math
import random
import time
from functools import cache
from typing import List

import numpy as np

from task_allocation.Task import TrajectoryTask


class agent:
    def __init__(
        self,
        state,
        environment,
        id,
        number_of_agents=None,
        capacity=None,
        tasks=None,
        color=None,
        point_estimation=False,
    ):
        self.environment = environment
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
        self.max_velocity = 3
        self.max_acceleration = 1

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
            self.set_state(state.coords[0])
        # socre function parameters
        self.Lambda = 0.95

        self.removal_list = {}
        self.removal_threshold = 15
        self.message_history = []

    def getPathTasks(self) -> List[TrajectoryTask]:
        result = []
        for task in self.path:
            result.append(self.tasks.get(task))
        return result

    def getTravelPath(self):
        assigned_tasks = self.getPathTasks()
        full_path = []
        if len(assigned_tasks) > 0:
            path, dist = self.environment.find_shortest_path(self.state, assigned_tasks[0].start, verify=False)
            full_path.extend(path)
            for i in range(len(assigned_tasks) - 1):
                path, dist = self.environment.find_shortest_path(assigned_tasks[i].end, assigned_tasks[i + 1].start, verify=False)
                full_path.extend(path)
            full_path.append(assigned_tasks[-1].end)
        return full_path

    def getPath(self):
        return self.path

    def getBundle(self):
        return self.bundle

    def set_state(self, state):
        self.state = state

    def send_message(self):
        return self.y, self.z, self.t

    def receive_message(self, Y):
        self.Y = Y

    def getTotalTravelCost(self, task_list: List[TrajectoryTask]):
        total_cost = 0
        if len(task_list) != 0:
            # Add the cost of travelling to the first task
            total_cost = self.getTravelCost(self.state, task_list[0].start)
            for t_index in range(len(task_list) - 1):
                total_cost += self.getTravelCost(task_list[t_index].end, task_list[t_index + 1].start)
            for t_index in range(len(task_list)):
                total_cost += self.getTravelCost(task_list[t_index].start, task_list[t_index].end)
            # Add the cost of returning home
            total_cost += self.getTravelCost(self.state, task_list[-1].end)
        return total_cost

    # This is only used for evaluations!
    def getTotalPathCost(self):
        finalTaskList = self.getPathTasks()
        total_dist = 0
        total_task_length = 0
        if len(finalTaskList) != 0:
            # Add the cost of travelling to the first task
            total_dist = np.linalg.norm(np.array(self.state) - np.array(finalTaskList[0].start))
            for t_index in range(len(finalTaskList) - 1):
                total_dist += np.linalg.norm(np.array(finalTaskList[t_index].end) - np.array(finalTaskList[t_index + 1].start))
            for t_index in range(len(finalTaskList)):
                total_task_length += np.linalg.norm(np.array(finalTaskList[t_index].start) - np.array(finalTaskList[t_index].end))
            # Add the cost of returning home
            total_dist += np.linalg.norm(np.array(self.state) - np.array(finalTaskList[-1].end))
            # Add the total task length
            total_dist += total_task_length
        return total_dist, total_task_length

    @cache
    def getTravelCost(self, start, end):
        # TODO move the cost calculations to the graph creation, then this function can be simplified to sum the costs of the path
        path, dist = self.environment.find_shortest_path(start, end, verify=False)

        # Travelcost in seconds
        # This is a optimised way of calculating euclidean distance: https://stackoverflow.com/questions/37794849/efficient-and-precise-calculation-of-the-euclidean-distance
        # dist = [(a - b) ** 2 for a, b in zip(start, end)]
        # dist = math.sqrt(sum(dist))
        # result = dist / self.max_velocity

        # Velocity ramp
        d_a = (self.max_velocity**2) / self.max_acceleration
        result = math.sqrt(4 * dist / self.max_acceleration) if dist < d_a else self.max_velocity / self.max_acceleration + dist / self.max_velocity

        return result  # the cost of travelling in seconds!

    def getTimeDiscountedReward(self, cost, task: TrajectoryTask):
        return self.Lambda ** (cost) * task.reward

    # S_i calculation of the agent
    def calculatePathReward(self):
        S_p = 0
        if len(self.path) > 0:
            travel_cost = self.getTravelCost(self.state, self.tasks[self.path[0]].start)
            S_p += self.Lambda ** (travel_cost) * self.tasks[self.path[0]].reward
            for p_idx in range(len(self.path) - 1):
                travel_cost += self.getTravelCost(self.tasks[self.path[p_idx]].end, self.tasks[self.path[p_idx + 1]].start)
                S_p += self.getTimeDiscountedReward(travel_cost, self.tasks[self.path[p_idx]])
        return S_p

    def getMinTravelCost(self, point, task: TrajectoryTask):
        distArray = [
            self.getTravelCost(point, task.start),
            self.getTravelCost(point, task.end),
        ]
        minIndex = np.argmin(distArray)
        shouldBeReversed = False
        if distArray[0] > distArray[1]:
            shouldBeReversed = True
        return distArray[minIndex], shouldBeReversed

    # Calculate the path reward with task j at index n
    def calculatePathRewardWithNewTask(self, j, n):
        temp_path = list(self.path)
        temp_path.insert(n, j)
        is_reversed = False
        # travel cost to first task
        travel_cost = self.getTravelCost(self.state, self.tasks[temp_path[0]].start)
        S_p = self.getTimeDiscountedReward(travel_cost, self.tasks[temp_path[0]])

        for p_idx in range(len(temp_path) - 1):
            if p_idx == n - 1:
                # The task is inserted at n, when evaluating the task use n-1 to determine whether it should be reversed
                temp_cost, is_reversed = self.getMinTravelCost(self.tasks[temp_path[p_idx]].end, self.tasks[temp_path[p_idx + 1]])
                travel_cost += temp_cost
            else:
                travel_cost += self.getTravelCost(self.tasks[temp_path[p_idx]].end, self.tasks[temp_path[p_idx + 1]].start)
            S_p += self.getTimeDiscountedReward(travel_cost, self.tasks[temp_path[p_idx]])

        # Add the cost for returning home
        travel_cost += self.getTravelCost(self.tasks[temp_path[-1]].end, self.state)
        S_p += self.getTimeDiscountedReward(travel_cost, self.tasks[temp_path[-1]])
        return S_p, is_reversed

    def getCij(self):
        """
        Returns the cost list c_ij for agent i where the position n results in the greatest reward
        """
        # Calculate Sp_i
        S_p = self.calculatePathReward()
        # init
        best_pos = None
        c = 0
        reverse = None
        best_task = None
        # try all tasks
        for j, task in self.tasks.items():
            # If already in the bundle list
            if j in self.bundle:
                c = 0  # Minimum Score
            else:
                # for each j calculate the path reward at each location in the local path
                for n in range(len(self.path) + 1):
                    S_pj, should_be_reversed = self.calculatePathRewardWithNewTask(j, n)
                    c_ijn = S_pj - S_p
                    if c <= c_ijn and c > self.y.get(j, -1):
                        c = c_ijn  # Store the cost
                        best_pos = n
                        reverse = should_be_reversed
                        best_task = j

        # reverse the task with max reward if necesarry
        if reverse:
            self.tasks[j].reverse()

        return best_task, best_pos, c

    def build_bundle(self):
        bundle_time = time.monotonic()
        while self.getTotalTravelCost(self.getPathTasks()) <= self.capacity:
            J_i, n_J, c = self.getCij()
            if J_i is None:
                break
            self.bundle.append(J_i)
            self.path.insert(n_J, J_i)

            self.y[J_i] = c
            self.z[J_i] = self.id
            self.t[J_i] = bundle_time  # Update the time of the winning bet

    def __update_time(self, task):
        self.t[task] = time.monotonic()

    def __action_rule(self, k, task, z_kj, y_kj, t_kj, z_ij, y_ij, t_ij, sender_info):
        eps = 5
        i = self.id
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
                    return {"y": self.y, "z": self.z, "t": self.t}

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
                    return {"y": self.y, "z": self.z, "t": self.t}

                elif y_kj == y_ij:
                    self.__leave()
                    return {"y": self.y, "z": self.z, "t": self.t}

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
                return {"y": self.y, "z": self.z, "t": self.t}

            elif z_ij == -1:
                self.__leave(task)
                return {"y": self.y, "z": self.z, "t": self.t}

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
                    return {"y": self.y, "z": self.z, "t": self.t}

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
                    return {"y": self.y, "z": self.z, "t": self.t}
                elif y_kj == y_ij:
                    self.__leave()
                    return {"y": self.y, "z": self.z, "t": self.t}
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
                return self.__leave()
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
        # Default leave and rebroadcast
        return self.__leave()

    def __rebroadcast(self, information):
        y = information["y"]
        z = information["z"]
        t = information["t"]
        # TODO
        self.__send_information(y, z, t)

    def __receive_information(self):
        message = self.my_socket.recieve(self.agent)
        if message is None:
            return None
        return message

    def __send_information(self, y, z, t, k=None):
        msg = {self.agent: {"y": y, "z": z, "t": t}}
        self.my_socket.send(self.agent, msg, k)

    def update_task(self):
        if self.Y is None:
            return

        self.message_history = []
        # Update Process
        update = 0
        for k in self.Y:
            for j in self.tasks:
                # Recieve info
                y_kj = self.Y[k][0].get(j, 0)  # Winning bids
                z_kj = self.Y[k][1].get(j, -1)  # Winning agent
                t_kj = self.Y[k][2].get(j, 0)  # Timestamps
                sender_info = {"y": self.Y[k][0], "z": self.Y[k][1], "t": self.Y[k][2]}

                # Own info
                y_ij = self.y.get(j, 0)
                z_ij = self.z.get(j, -1)
                t_ij = self.t.get(j, 0)

                rebroadcast = self.__action_rule(
                    k=k, task=j, z_kj=z_kj, y_kj=y_kj, t_kj=t_kj, z_ij=z_ij, y_ij=y_ij, t_ij=t_ij, sender_info=sender_info
                )
                if rebroadcast:
                    # self.__rebroadcast(rebroadcast)
                    update += 1
                    # print({"a": k, "y": y_kj, "z": z_kj, "t": t_kj, "task": j, "i": self.id, "y_i": y_ij, "z_i": z_ij, "t_i": t_ij})
                    self.message_history.append(
                        {"a": k, "y": y_kj, "z": z_kj, "t": t_kj, "task": j, "i": self.id, "y_i": y_ij, "z_i": z_ij, "t_i": t_ij}
                    )
        return update

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

        removal_list = self.bundle[index:]
        # TODO make sure to reimplement removel threshold
        # self.removal_list[self.bundle[index]] = self.removal_list[self.bundle[index]] + 1
        self.path = [num for num in self.path if num not in removal_list]
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
        return {"y": self.y, "z": self.z, "t": self.t}
