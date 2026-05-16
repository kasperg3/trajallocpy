"""Performance Impact (PI) task allocation, adapted to trajectory tasks.

Decentralized PI after Zhao, Meng & Chung, "A heuristic distributed task
allocation method for multivehicle multitask problems and its application to
UAVs" (IEEE T-Cybernetics, 2016). Unlike CBBA, which bids the *maximum
marginal utility*, PI greedily includes the task with the smallest increase in
the agent's total travel/time cost (its "performance impact") and resolves
conflicts in favor of the agent with the lowest impact. This module mirrors the
``CBBA.agent`` API so it is a drop-in for ``Experiment.Runner``.
"""

import copy
import multiprocessing
import random
import time
from typing import List

import numpy as np

from trajallocpy import Agent
from trajallocpy.Task import TrajectoryTask

EPSILON = np.finfo(float).eps
# A task that no agent has feasibly included has the worst possible bid.
NO_BID = -np.inf


class BundleResult:
    def __init__(self, agent):
        self.bundle = agent.bundle
        self.path = agent.path
        self.winning_agents = agent.winning_agents
        self.winning_bids = agent.winning_bids
        self.timestamps = agent.timestamps
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
        self.color = color if color is not None else (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))

        self.max_velocity = 3
        self.max_acceleration = 1
        self.id = id

        # winning_bids holds -performance_impact (higher == lower impact == better).
        self.winning_agents = np.full(self.task_num, -1, dtype=np.int64)
        self.winning_bids = np.full(self.task_num, NO_BID, dtype=np.float64)
        self.bundle = []
        self.path = []
        self.times = []

        if capacity is None:
            raise Exception("Error: agent capacity cannot be None")
        self.capacity = capacity

        self.time_step = time.monotonic()
        self.timestamps = {a: self.time_step for a in range(number_of_agents)}

        if state is None:
            raise Exception("ERROR: Initial state cannot be None")
        self.state = state.coords[0]
        self.Lambda = 0.99
        self.availability_time = 0

        self.removal_list = np.zeros(self.task_num, dtype=np.int64)
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

    def _path_cost(self, task_indices) -> float:
        return Agent.getTotalTravelCost(self.state, [self.tasks[i] for i in task_indices], self.environment)

    def _is_feasible(self, task_indices) -> bool:
        if self._path_cost(task_indices) > self.capacity:
            return False
        return Agent.countTimeWindowViolations(self.state, [self.tasks[i] for i in task_indices], self.environment) == 0

    def _best_impact(self, j, base_cost):
        """Minimum cost increase of inserting task j, with orientation/position."""
        best = (np.inf, 0, False)  # (impact, position, reversed)
        for n in range(len(self.path) + 1):
            candidate = self.path[:]
            candidate.insert(n, j)
            for reverse in (False, True):
                if reverse:
                    self.tasks[j].reverse()
                feasible = self._is_feasible(candidate)
                cost = self._path_cost(candidate)
                if reverse:
                    self.tasks[j].reverse()  # restore original orientation
                if not feasible:
                    continue
                impact = cost - base_cost
                if impact < best[0]:
                    best = (impact, n, reverse)
        return best

    def build_bundle(self, queue: multiprocessing.Queue = None):
        ignore = {k for k, v in enumerate(self.removal_list) if v > self.removal_threshold}
        while True:
            if self._path_cost(self.path) > self.capacity:
                break
            base_cost = self._path_cost(self.path)
            candidates = set(range(self.task_num)).difference(self.bundle).difference(ignore)

            best_task, best_impact, best_pos, best_rev = None, np.inf, 0, False
            for j in candidates:
                impact, pos, rev = self._best_impact(j, base_cost)
                if not np.isfinite(impact):
                    continue
                my_bid = -impact
                holder = self.winning_agents[j]
                # Skip a task another agent already holds unless we strictly
                # beat it (lower impact, or tie broken by lower agent id).
                if holder not in (-1, self.id):
                    if my_bid < self.winning_bids[j] - EPSILON:
                        continue
                    if abs(my_bid - self.winning_bids[j]) <= EPSILON and self.id >= holder:
                        continue
                if impact < best_impact:
                    best_task, best_impact, best_pos, best_rev = j, impact, pos, rev

            if best_task is None or not np.isfinite(best_impact):
                break

            my_bid = -best_impact
            if best_rev:
                self.tasks[best_task].reverse()

            self.bundle.append(best_task)
            self.path.insert(best_pos, best_task)
            self.times = Agent.getArrivalTimes(self.state, self.getPathTasks(), self.environment)
            self.winning_bids[best_task] = my_bid
            self.winning_agents[best_task] = self.id

        if queue is not None:
            queue.put(BundleResult(self))
        else:
            return BundleResult(self)

    def update_task(self, messages):
        id_list = list(messages.keys())
        time_now = time.monotonic()
        for agent_id in list(self.timestamps.keys()):
            if agent_id in id_list:
                self.timestamps[agent_id] = time_now
            else:
                s_list = [messages[n][2][agent_id] for n in id_list]
                if s_list:
                    self.timestamps[agent_id] = max(s_list)

        self.time_step += 1
        conflicts = 0
        for j in range(self.task_num):
            # The task goes to the agent with the lowest performance impact
            # (== highest bid). Ties break toward the lower agent id. Starting
            # from our own belief and folding in every neighbor's makes this a
            # global argmax that converges once messages have propagated.
            best_y = self.winning_bids[j]
            best_z = self.winning_agents[j]
            for k in id_list:
                y_k, z_k, _ = messages[k]
                z_kj, y_kj = z_k[j], y_k[j]
                if z_kj == -1:
                    continue
                if best_z == -1:
                    better = True
                elif y_kj > best_y + EPSILON:
                    better = True
                elif abs(y_kj - best_y) <= EPSILON and z_kj < best_z:
                    better = True
                else:
                    better = False
                if better:
                    best_y, best_z = y_kj, z_kj
            if best_z != self.winning_agents[j] or abs(best_y - self.winning_bids[j]) > EPSILON:
                conflicts += self.__update(j, best_y, best_z)
        return conflicts

    def __update_path(self, task):
        # PI keeps an order-independent list: losing one task removes only that
        # task (unlike CBBA, whose ordered bids force releasing the whole tail).
        if task not in self.bundle:
            return 0
        self.removal_list[task] = self.removal_list[task] + 1
        self.bundle = [t for t in self.bundle if t != task]
        self.path = [t for t in self.path if t != task]
        self.times = Agent.getArrivalTimes(self.state, self.getPathTasks(), self.environment)
        return 1

    def __update(self, j, y_kj, z_kj):
        self.winning_bids[j] = y_kj
        self.winning_agents[j] = z_kj
        return self.__update_path(j)
