#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class Task:
    task_id: int = 0
    task_value: float = 0  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    discount: float = 0.1  # task exponential discount (lambda)


class PointTask(Task):
    task_type: int = 0
    x0: float = 0
    y0: float = 0


class CoverageTask(Task):
    task_type: int = 1
    x0: float = 0
    y0: float = 0
    x1: float = 0
    y1: float = 0
