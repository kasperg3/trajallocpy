import math
from typing import List, Tuple

Point = Tuple[float, float]
Line = Tuple[Point, Point]


def angle_between_points(p1: Point, p2: Point) -> float:
    """Calculates the angle between two points"""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def intersect(line1: Line, line2: Line) -> Point:
    """Calculates the intersection point of two lines"""
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    x_num = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
    y_num = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if den == 0:
        return None

    x = x_num / den
    y = y_num / den
    return (x, y)


def is_visible(point: Point, start: Point, end: Point, obstacles: List[Line]) -> bool:
    """Checks if a point is visible from a given start point, end point, and obstacle lines"""
    if start == point or end == point:
        return False

    for obstacle in obstacles:
        if intersect((start, point), obstacle) is not None:
            return False
        if intersect((end, point), obstacle) is not None:
            return False

    return True


def visible_vertices(vertices: List[Point], obstacles: List[Line]) -> List[Point]:
    """Calculates the list of visible vertices from a given list of vertices and obstacle lines"""
    visible = []
    n = len(vertices)

    for i in range(n):
        start = vertices[i]
        end = vertices[(i + 1) % n]
        angle_start = angle_between_points(start, end)

        visible.append(start)
        visible.append(end)

        for j in range(n):
            if j != i and j != (i + 1) % n:
                point = vertices[j]
                angle = angle_between_points(start, point)
                if angle > angle_start and is_visible(point, start, end, obstacles):
                    visible.append(point)
    return visible


if __name__ == "__main__":
    vertices = [(0, 0), (0, 1), (1, 1), (1, 0)]
    obstacles = [((0.5, 0.5), (0.5, 1)), ((0.5, 0.5), (1, 0.5))]
    visible = visible_vertices(vertices, obstacles)
    print(visible)
