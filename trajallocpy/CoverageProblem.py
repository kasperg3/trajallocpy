import random

import shapely.geometry
from extremitypathfinder import PolygonEnvironment

from trajallocpy import Task


class CoverageProblem:
    def __init__(
        self,
        tasks,
        search_area: shapely.Polygon,
        restricted_areas: shapely.geometry.MultiPolygon,
    ):
        self.__restricted_areas = restricted_areas
        self.__search_area = search_area

        # TODO Use extremity planner to save the graph
        self.environment = PolygonEnvironment()
        holes = []
        for polygon in restricted_areas.geoms:
            # Properly orient the obstacle polygons
            holes.append(list(shapely.geometry.polygon.orient(polygon, -1).exterior.coords[:-1]))
        shapely.geometry.polygon.orient(search_area, 1.0)

        self.environment.store(list(shapely.geometry.polygon.orient(search_area, 1.0).exterior.coords[:-1]), holes, validate=True)

        task_list = []
        for id, trajectory in enumerate(tasks.geoms):
            task_list.append(Task.TrajectoryTask(id, trajectory))

        self.__tasks = task_list

    def getRestrictedAreas(self):
        return self.__restricted_areas

    def getSearchArea(self):
        return self.__search_area

    def getTasks(self):
        return self.__tasks

    def getNumberOfTasks(self):
        return len(self.__tasks)

    def generate_random_point_in_problem(self) -> shapely.geometry.Point:
        minx, miny, maxx, maxy = self.__search_area.bounds
        while True:
            point = shapely.geometry.Point(random.uniform(minx, maxx), random.uniform(miny, maxy))  # noqa: S311
            if not self.__restricted_areas.contains(point) and self.__search_area.contains(point):
                return point
