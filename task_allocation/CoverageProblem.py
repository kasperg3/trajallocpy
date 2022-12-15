class CoverageProblem:
    def __init__(self, restricted_area, search_area, sweeps):
        self.__restricted_area = restricted_area
        self.__search_area = search_area
        self.__sweeps = sweeps

    def getRestrictedArea(self):
        return self.__restricted_area

    def getSearchArea(self):
        return self.__search_area

    def getSweeps(self):
        return self.__sweeps
