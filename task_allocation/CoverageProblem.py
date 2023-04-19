import math
import random

import networkx as nx
import numpy as np
import shapely.geometry
from geojson import FeatureCollection

from task_allocation import Task, VisibilityGraph


class CoverageProblem:
    def __init__(self, geojson: FeatureCollection, number_of_robots):
        geometries = {
            "obstacles": shapely.geometry.MultiPolygon(),
            "tasks": shapely.geometry.MultiLineString(),
            "boundary": shapely.geometry.Polygon(),
        }
        for feature in geojson:
            if feature["geometry"]:
                geometries[feature["id"]] = shapely.geometry.shape(feature["geometry"])
        self.__restricted_areas = geometries["obstacles"]
        self.__search_area = geometries["boundary"]

        # Create a GeometryCollection with the geometries and their types
        self.travel_graph = VisibilityGraph.naive_visibility_graph(
            geometries["boundary"], geometries["obstacles"], reduced_visibility=False
        )

        start_points = [trajectory.coords[0] for trajectory in list(geometries["tasks"].geoms)]
        end_points = [trajectory.coords[-1] for trajectory in list(geometries["tasks"].geoms)]
        start_points.extend(end_points)

        # Add all the task endpoints
        VisibilityGraph.add_points_to_graph(self.travel_graph, start_points)
        # Add a cost based on the euclidean distance for each edge
        edge_attributes = {
            e: math.sqrt(sum([(a - b) ** 2 for a, b in zip(e[0], e[1])])) for e in self.travel_graph.edges()
        }
        nx.set_edge_attributes(
            G=self.travel_graph,
            values=edge_attributes,
            name="cost",
        )
        print("Travel graph ", self.travel_graph)

        # convert geometries to tasks
        tasks = []
        for id, trajectory in enumerate(geometries["tasks"].geoms):
            tasks.append(Task.TrajectoryTask(id, trajectory, trajectory.coords[0], trajectory.coords[-1]))

        self.__tasks = tasks
        self.__n_robots = number_of_robots
        self.__com_graph = np.ones((number_of_robots, number_of_robots))

    def getRestrictedAreas(self):
        return self.__restricted_areas

    def getSearchArea(self):
        return self.__search_area

    def getTasks(self):
        return self.__tasks

    def getNumberOfTasks(self):
        return len(self.__tasks)

    def setCommunicationGraph(self, com_graph):
        self.__com_graph = com_graph

    def getCommunicationGraph(self):
        return self.__com_graph

    def getNumberOfRobots(self):
        return self.__n_robots

    def setNumberOfRobots(self, n):
        self.__n_robots = n

    def generate_random_point_in_problem(self) -> shapely.geometry.Point:
        minx, miny, maxx, maxy = self.__search_area.bounds
        while True:
            point = shapely.geometry.Point(random.uniform(minx, maxx), random.uniform(miny, maxy))  # noqa: S311
            if not self.__restricted_areas.contains(point) and self.__search_area.contains(point):
                return point
                return point
