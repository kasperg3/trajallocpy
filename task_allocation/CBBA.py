import copy
import random

import numpy as np

from task_allocation import Task


class agent:
    def __init__(self, id=None, task_num=None, agent_num=None, L_t=None, state=None):
        self.task_num = task_num
        self.task_idx = [j for j in range(self.task_num)]

        self.agent_num = agent_num
        self.color = (
            random.uniform(0, 1),
            random.uniform(0, 1),
            random.uniform(0, 1),
        )
        self.velocity = 1

        # Agent ID
        self.id = id

        # Local Winning Agent List
        self.winning_agents = np.ones(self.task_num, dtype=np.int8) * self.id
        # Local Winning Bid List
        self.winning_bids = np.array(
            [0 for _ in range(self.task_num)], dtype=np.float64
        )
        # Bundle
        self.bundle = []
        # Path
        self.path = []
        # Maximum Task Number #TODO add an option to have non integer task limits
        self.L_t = L_t

        # Local Clock
        self.time_step = 0
        # Time Stamp List
        self.timestamps = {a: self.time_step for a in range(self.agent_num)}

        # initialize state
        if state is None:
            self.set_state(
                np.random.uniform(low=0, high=0.1, size=(1, 2)).squeeze()
            )  # Agent State (Position)
        else:
            self.set_state(state.squeeze())
        self.c = np.zeros(self.task_num)  # Initial Score (Euclidean Distance)
        # socre function parameters
        self.Lambda = 0.95

    def tau(self, j):
        pass

    def set_state(self, state):
        self.state = state

    def send_message(self):
        return self.winning_bids.tolist(), self.winning_agents.tolist(), self.timestamps

    def receive_message(self, Y):
        self.Y = Y

    def euclideanDist(self, p0, p1):
        return np.linalg.norm(p0 - p1)

    def getMinDistance(self, robot_state, task):
        distArray = [
            self.euclideanDist(robot_state, task.start),
            self.euclideanDist(robot_state, task.end),
        ]
        minIndex = np.min(distArray)
        return (minIndex, distArray[minIndex])

    def getTravelCost(self, start, end):
        # TODO use task type to determine the
        return np.linalg.norm(end - start) / self.vel

    def getTimeDiscountedReward(self, cost, task):
        (self.Lambda ** (cost / self.vel)) * task.reward

    # S_i calculation of the agent
    def calculatePathReward(self):
        S_p = 0
        if len(self.path) > 0:
            for p_idx in range(len(self.path)):
                if p_idx == 0:
                    # The first task has to calculate the distance to the start position
                    travel_cost = self.getTravelCost(
                        self.state.squeeze(), self.path[p_idx].start
                    )
                    S_p += self.getTimeDiscountedReward(travel_cost, self.path[p_idx])
                else:
                    travel_cost = self.getTravelCost(
                        self.path[p_idx - 1].end, self.path[p_idx].start
                    )
                    S_p += self.getTimeDiscountedReward(travel_cost, self.path[p_idx])
        return S_p

    # Calculate the path reward with task j at index n
    def calculatePathRewardWithNewTask(self, tasks, j, n):
        S_p = 0
        # TODO implement this and add debugging
        return S_p

    def getCij(self, tasks):
        """
        Returns the cost list c_ij for agent i where the position n results in the greatest reward
        """
        # Calculate Sp_i
        S_p = self.calculatePathReward()
        best_pos = np.zeros(self.task_num)
        c = np.zeros(self.task_num)
        # TODO Calculate c_ij for each task j
        for j in range(self.task_num):
            if j in self.bundle:  # If already in bundle list
                c[j] = 0  # Minimum Score
            else:
                # for each j calculate the path reward at each location in the local path
                best_pos = 0
                for n in range(len(self.path) + 1):
                    c_ijn = self.calculatePathRewardWithNewTask(tasks, j, n) - S_p
                    if c[j] < c_ijn:
                        c[j] = c_ijn  # Store the cost
                        best_pos[j] = n
        return (best_pos, c)

    def build_bundle(self, tasks):
        while len(self.bundle) < self.L_t:
            best_pos, self.c = self.getCij(tasks)
            # TODO update self.c to c_ij
            h = self.c > self.winning_bids

            if sum(h) == 0:  # No valid task
                break
            self.c[~h] = 0
            # TODO This should
            J_i = np.argmax(self.c)
            n_J = best_pos[J_i]

            # TODO when inserting the task make sure to update the start/end
            self.bundle.append(J_i)
            self.path.insert(n_J, J_i)

            self.winning_bids[J_i] = self.c[J_i]
            self.winning_agents[J_i] = self.id

    def update_task(self):
        old_p = copy.deepcopy(self.path)
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
                        elif abs(y_kj - y_ij) < np.finfo(float).eps:  # Tie Breaker
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
                        if (s_k[m] > self.timestamps[m]) or (y_kj > y_ij):
                            self.__update(j, y_kj, z_kj)
                        elif abs(y_kj - y_ij) < np.finfo(float).eps:  # Tie Breaker
                            if k < self.id:
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
                        if (s_k[m] >= self.timestamps[m]) and (y_kj > y_ij):
                            self.__update(j, y_kj, z_kj)
                        # Tie Breaker
                        elif (s_k[m] >= self.timestamps[m]) and (
                            abs(y_kj - y_ij) < np.finfo(float).eps
                        ):
                            if m < self.id:
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
                        if (s_k[m] > self.timestamps[m]) and (
                            s_k[n] > self.timestamps[n]
                        ):
                            self.__update(j, y_kj, z_kj)
                        elif (s_k[m] > self.timestamps[m]) and (y_kj > y_ij):
                            self.__update(j, y_kj, z_kj)
                        elif (s_k[m] > self.timestamps[m]) and (
                            abs(y_kj - y_ij) < np.finfo(float).eps
                        ):  # Tie Breaker
                            if m < n:
                                self.__update(j, y_kj, z_kj)
                        elif (s_k[n] > self.timestamps[n]) and (
                            self.timestamps[m] > s_k[m]
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

        n_bar = len(self.bundle)
        # Get n_bar
        for n in range(len(self.bundle)):
            b_n = self.bundle[n]
            if self.winning_agents[b_n] != self.id:
                n_bar = n
                break

        b_idx1 = copy.deepcopy(self.bundle[n_bar + 1 :])

        if len(b_idx1) > 0:
            self.winning_bids[b_idx1] = 0
            self.winning_agents[b_idx1] = -1

        if n_bar < len(self.bundle):
            del self.bundle[n_bar:]

        self.time_step += 1

        converged = False
        if old_p == self.path:
            converged = True

        return converged

    def __update(self, j, y_kj, z_kj):
        """
        Update values
        """
        self.winning_bids[j] = y_kj
        self.winning_agents[j] = z_kj

    def __reset(self, j):
        """
        Reset values
        """
        self.winning_bids[j] = 0
        self.winning_agents[j] = -1  # -1 means "none"

    def __leave(self):
        """
        Do nothing
        """
        pass


class consensus_algorithm:
    def __init__(self) -> None:
        pass

    def solve():
        pass
