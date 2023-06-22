#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class agent:
    id: int
    position: list
    capacity: int
    max_velocity: float = 3  # m/s
    max_acceleration: float = 1  # m/s^2
