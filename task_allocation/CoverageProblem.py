from typing import List

import numpy as np
import shapely.geometry
from geojson import FeatureCollection

from task_allocation import Task


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
        self.__tasks = geometries["tasks"]
        self.__n_robots = number_of_robots
        self.__com_graph = np.ones((number_of_robots, number_of_robots))

    def getRestrictedAreas(self):
        return self.__restricted_areas

    def getSearchArea(self):
        return self.__search_area

    def getTasks(self):
        return self.__tasks

    def setCommunicationGraph(self, com_graph):
        self.__com_graph = com_graph

    def getCommunicationGraph(self):
        return self.__com_graph

    def getNumberOfRobots(self):
        return self.__n_robots

    def setNumberOfRobots(self, n):
        self.__n_robots = n
