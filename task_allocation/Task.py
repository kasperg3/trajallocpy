#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List


@dataclass
class Task:
    task_id: int
    start: List[float]
    end: List[float]
    reward: float = 1  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    discount: float = 0.1  # task exponential discount (lambda)
    task_type: int = 1
