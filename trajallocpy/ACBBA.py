import copy
import itertools
import multiprocessing
import random
import time
from typing import List

import numpy as np

import trajallocpy.Agent as Agent
from trajallocpy.Task import TrajectoryTask

EPSILON = 1e-6
# Tolerance for comparing message timestamps. ``np.finfo(float).eps`` is far too
# tight once timestamps come from a wall/simulated clock in seconds: ties would
# never be detected and the consensus would livelock.
TIME_EPS = 1e-6


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
        capacity=0,
        environment=None,
        tasks=None,
        color=None,
        point_estimation=False,
        max_velocity=3,
        max_acceleration=1,
        Lambda=None,
        removal_threshold=5,
        clock=time.monotonic,
    ):
        self._clock = clock
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
        self.winning_agents = {}
        # Local Winning Bid List
        self.winning_bids = {}
        # Time Stamp List
        self.t = {}
        self.times = []
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
        # score function parameters
        self.Lambda = 0.95 if Lambda is None else Lambda

        self.removal_list = {}
        self.removal_threshold = removal_threshold
        self.message_history = []

    def __repr__(self) -> str:
        return f"Agent {self.id} \n path {self.path} \n  bundle {self.bundle} \n y(winning bids) {self.winning_bids} \n z(winning agents) {self.winning_agents} \n t(timestamps) {self.t} \n"

    def add_tasks(self, tasks):
        # add the tasks to self.tasks dictionary
        for task in tasks:
            self.tasks[task.id] = task

    def __str__(self) -> str:
        return f"Agent {self.id} \n path {self.path} \n  bundle {self.bundle} \n y(winning bids) {self.winning_bids} \n z(winning agents) {self.winning_agents} \n t(timestamps) {self.t} \n"

    def getPathTasks(self) -> List[TrajectoryTask]:
        result = []
        for task in self.path:
            result.append(self.tasks.get(task))
        return result

    def send_message(self):
        # Only broadcast tasks this agent has an opinion about (it has bid on
        # one, or relayed/learned a winner/timestamp). Emitting the default
        # ``y=0, z=-1, t=0`` for every task is pure noise that inflates message
        # volume and slows convergence.
        known = set(self.winning_bids) | set(self.winning_agents) | set(self.t)
        return [
            Agent.BidInformation(
                y=self.winning_bids.get(task_id, 0),
                z=self.winning_agents.get(task_id, -1),
                t=self.t.get(task_id, 0),
                j=task_id,
                k=self.id,
            )
            for task_id in known
        ]

    def getCij(self):
        """
        Returns the cost list c_ij for agent i where the position n results in the greatest reward
        """
        # Calculate Sp_i
        S_p = Agent.calculatePathReward(self.state, self.getPathTasks(), self.environment, self.capacity, self.Lambda)
        # init (tasks are keyed by task.id throughout ACBBA)
        best_pos = {task_id: 0 for task_id in self.tasks}
        c = {task_id: -np.inf for task_id in self.tasks}
        reverse = {task_id: 0 for task_id in self.tasks}
        best_time = {task_id: 0 for task_id in self.tasks}

        # Collect the tasks which should be considered for planning. removal_list
        # is keyed by task id, so iterate items (not enumerate).
        ignore_tasks = {k for k, v in self.removal_list.items() if v > self.removal_threshold}
        tasks_to_check = set(self.tasks).difference(self.bundle).difference(ignore_tasks)

        for n, j in itertools.product(range(len(self.path) + 1), tasks_to_check):
            S_pj, should_be_reversed, time_to_task, feasible = Agent.calculatePathRewardWithNewTask(
                j, n, self.state, self.tasks, self.path, self.environment, self.Lambda, self.capacity, self.use_single_point_estimation
            )
            if not feasible:  # hard time-window violation: never select this insertion
                continue
            c_ijn = S_pj - S_p
            if c_ijn > c[j]:
                c[j] = c_ijn  # Store the cost
                best_pos[j] = n
                reverse[j] = should_be_reversed
                best_time[j] = time_to_task

        return (best_pos, c, reverse, best_time)

    def build_bundle(self, queue: multiprocessing.Queue = None):
        # DMG warp: bids must be non-increasing along the bundle so the scoring
        # is a diminishing-marginal-gain function (Choi et al. 2009), which is
        # what guarantees convergence.
        last_bid = self.winning_bids.get(self.bundle[-1], np.inf) if self.bundle else np.inf
        while Agent.getTotalTravelCost(self.state, self.getPathTasks(), self.environment) <= self.capacity:
            best_pos, c, reverse, best_time = self.getCij()

            best_task, best_value = None, -np.inf
            for task_id, value in c.items():
                if not np.isfinite(value):
                    continue
                y_ij = self.winning_bids.get(task_id, 0)
                z_ij = self.winning_agents.get(task_id, -1)
                outbids = value - y_ij > EPSILON
                tie_win = abs(value - y_ij) <= EPSILON and (z_ij == -1 or self.id < z_ij)
                if (outbids or tie_win) and value > best_value:
                    best_task, best_value = task_id, value
            if best_task is None:  # No valid task
                break

            n_J = best_pos[best_task]
            if reverse[best_task]:
                self.tasks[best_task].reverse()

            # Check for capacity before committing the task.
            potential_path = self.path[:]
            potential_path.insert(n_J, best_task)
            potential_capacity = Agent.getTotalTravelCost(self.state, [self.tasks[i] for i in potential_path], self.environment)
            if potential_capacity > self.capacity:
                break

            self.bundle.append(best_task)
            self.path.insert(n_J, best_task)
            self.times = Agent.getArrivalTimes(self.state, self.getPathTasks(), self.environment)

            warped_bid = min(best_value, last_bid)
            self.winning_bids[best_task] = warped_bid
            self.winning_agents[best_task] = self.id
            self.t[best_task] = self._clock()
            last_bid = warped_bid

        if queue is not None:
            queue.put(BundleResult(self))
        else:
            return BundleResult(self)

    def __update_time(self, task):
        self.t[task] = self._clock()

    def __action_rule(self, k, j, task, z_kj, y_kj, t_kj, z_ij, y_ij, t_ij) -> Agent.BidInformation:
        eps = TIME_EPS
        i = self.id
        sender_info = Agent.BidInformation(y=y_kj, z=z_kj, t=t_kj, j=j, k=self.id)
        own_info = Agent.BidInformation(y=y_ij, z=z_ij, t=t_ij, j=j, k=self.id)
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
                    # rebuild so the relayed bid carries the bumped timestamp
                    return Agent.BidInformation(y=y_ij, z=z_ij, t=self.t[task], j=j, k=self.id)

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
                    # rebuild so the relayed bid carries the bumped timestamp
                    return Agent.BidInformation(y=y_ij, z=z_ij, t=self.t[task], j=j, k=self.id)

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

    def receive(self, mailbox) -> List[Agent.BidInformation]:
        """Drain an inbox of incoming bid batches and run the consensus rules.

        Returns the list of :class:`Agent.BidInformation` that should be
        rebroadcast to neighbours (the worker forwards them via the transport).
        """
        rebroadcasts = []
        for batch in mailbox.get_nowait_all():
            rebroadcasts.extend(self.update_task_async(batch))
        return rebroadcasts

    def update_task_async(self, bids: List[Agent.BidInformation]):
        # Update Process
        rebroadcasts = []
        for bid_info in bids:
            j = bid_info.j
            k = bid_info.k

            # Own info
            y_ij = self.winning_bids.get(j, 0)
            z_ij = self.winning_agents.get(j, -1)
            t_ij = self.t.get(j, 0)

            # Recieved info
            y_kj = bid_info.y  # Winning bids
            z_kj = bid_info.z  # Winning agent
            t_kj = bid_info.t  # Timestamps

            rebroadcast = self.__action_rule(k=k, j=j, task=j, z_kj=z_kj, y_kj=y_kj, t_kj=t_kj, z_ij=z_ij, y_ij=y_ij, t_ij=t_ij)
            if rebroadcast is not None:
                rebroadcasts.append(rebroadcast)
        return rebroadcasts

    def __update(self, y_kj, z_kj, t_kj, j):
        """
        Update values
        """
        self.winning_bids[j] = y_kj
        self.winning_agents[j] = z_kj
        self.t[j] = t_kj
        self.__update_path(j)

    def __update_path(self, task):
        if task not in self.bundle:
            return
        index = self.bundle.index(task)
        b_retry = self.bundle[index + 1 :]
        for idx in b_retry:
            self.winning_bids[idx] = 0
            self.winning_agents[idx] = -1
            # Do NOT advance self.t here: bumping the timestamp on a cascade
            # release lets the releasing agent look "newer" and immediately
            # reclaim the task, which causes livelock under async delivery.

        self.removal_list[task] = self.removal_list.get(task, 0) + 1
        self.path = [num for num in self.path if num not in self.bundle[index:]]
        self.bundle = self.bundle[:index]

    def __reset(self, task):
        self.winning_bids[task] = 0
        self.winning_agents[task] = -1
        self.t[task] = self._clock()
        self.__update_path(task)

    def __leave(self):
        """
        Do nothing
        """
        return
