import random

import shapely.geometry
from geojson import FeatureCollection

from task_allocation import Task, VisibilityGraph


class CoverageProblem:
    def __init__(
        self,
        tasks,
        search_area: shapely.geometry,
        restricted_areas: shapely.geometry.MultiPolygon,
    ):
        self.__restricted_areas = restricted_areas
        self.__search_area = search_area

        # Create a GeometryCollection with the geometries and their types
        self.travel_graph = VisibilityGraph.visibility_graph(search_area, restricted_areas, reduced_visibility=False)

        # Add all the task endpoints
        start_points = [trajectory.coords[0] for trajectory in list(tasks.geoms)]
        end_points = [trajectory.coords[-1] for trajectory in list(tasks.geoms)]
        start_points.extend(end_points)
        VisibilityGraph.add_points_to_graph(self.travel_graph, start_points)

        print("Travel graph ", self.travel_graph)
        task_list = []
        for id, trajectory in enumerate(tasks.geoms):
            task_list.append(Task.TrajectoryTask(id, trajectory, trajectory.coords[0], trajectory.coords[-1]))

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
                return point
