import numpy as np


class CoverageProblem:
    def __init__(self, restricted_area, search_area, sweeps, number_of_robots=4):
        self.__restricted_area = restricted_area
        self.__search_area = search_area
        self.__sweeps = sweeps
        self.__n_robots = number_of_robots
        # Initialize Fully connected communication network
        self.__com_graph = np.ones((number_of_robots, number_of_robots))

    def getRestrictedAreas(self):
        return self.__restricted_area

    def getSearchArea(self):
        return self.__search_area

    def getSweeps(self):
        return self.__sweeps

    def setCommunicationGraph(self, com_graph):
        self.__com_graph = com_graph

    def getCommunicationGraph(self):
        return self.__com_graph

    def getNumberOfRobots(self):
        return self.__n_robots
