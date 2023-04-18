#!/usr/bin/env python3
from dataclasses import dataclass

import shapely


@dataclass
class TrajectoryTask:
    task_id: int
    trajectory: shapely.LineString  # List of points
    start: shapely.Point
    end: shapely.Point
    reward: float = 1  # task reward
    start_time: float = 0  # task start time (sec)
    end_time: float = 0  # task expiry time (sec)
    duration: float = 0  # task default duration (sec)
    task_type: int = 1

    def reverse(self):
        self.trajectory = shapely.LineString(self.trajectory.coords[::-1])
        # re initialize the start and end
        self.end = self.trajectory.coords[0]
        self.start = self.trajectory.coords[-1]
