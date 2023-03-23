#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List


@dataclass
class TrajectoryTask:
    task_id: int
    trajectory: List[tuple]  # List of points
    reward: float = 1  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    task_type: int = 1

    def reverse(self):
        self.trajectory.reverse()

    def getStart(self):
        return self.trajectory[0]

    def getEnd(self):
        return self.trajectory[-1]

    def getReward(self):
        return self.reward
