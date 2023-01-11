import numpy as np


class CoverageProblem:
    def __init__(self, restricted_area, search_area, sweeps, number_of_robots=4):
        # TODO convert area dict to np array
        # TODO convert all lon/lat to utm coordinates
        self.__restricted_area = restricted_area
        self.__search_area = search_area
        self.__n_robots = number_of_robots
        # Initialize Fully connected communication network
        self.__com_graph = np.ones((number_of_robots, number_of_robots))
        tasks = []
        
        # TODO make this a line coverage task
        for i in range(len(sweeps) - 1):
            if not i % 2:
                lat = (
                    (sweeps[i]["latitude"] - sweeps[i + 1]["latitude"]) / 2
                ) + sweeps[i]["latitude"]
                lon = (
                    (sweeps[i]["longitude"] - sweeps[i + 1]["longitude"]) / 2
                ) + sweeps[i]["longitude"]
                tasks.append([lat, lon])

        self.__sweeps = np.array(tasks)

    def getRestrictedAreas(self):
        return self.__restricted_area

    def getSearchArea(self):
        return self.__search_area

    def getSweeps(self):
        # TODO return tuples of 2 coordinates
        return self.__sweeps

    def setCommunicationGraph(self, com_graph):
        self.__com_graph = com_graph

    def getCommunicationGraph(self):
        return self.__com_graph

    def getNumberOfRobots(self):
        return self.__n_robots

    def __convert_geodetic_to_UTM(coordinate):
        pass
