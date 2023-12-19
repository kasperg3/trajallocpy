#!/usr/bin/env python3
from dataclasses import dataclass

import shapely


@dataclass
class PointTask:
    id: int
    point: shapely.Point
    start: shapely.Point = None  # Accessing the start/end directly is 4x more efficient
    end: shapely.Point = None
    reward: float = 1  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    task_type: int = 1

    def __post_init__(self):
        self.start = self.point
        self.end = self.point


@dataclass
class TrajectoryTask:
    id: int
    trajectory: shapely.LineString  # List of points
    start: shapely.Point = None  # Accessing the start/end directly is 4x more efficient
    end: shapely.Point = None
    reward: float = 1  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    length: float = 0
    task_type: int = 1

    def __post_init__(self):
        self.start = self.trajectory.coords[0]
        self.end = self.trajectory.coords[-1]
        self.length = self.trajectory.length  # unitless length
        # TODO init the task cost/length/time

    def reverse(self):
        self.trajectory = shapely.LineString(self.trajectory.coords[::-1])
        # re initialize the start and end
        self.end = self.trajectory.coords[0]
        self.start = self.trajectory.coords[-1]
        
    
