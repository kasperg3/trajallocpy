import json
from task_allocation import Experiment, Utility
import numpy as np
import os


def run_experiment():

    seed = 8986413
    np.random.seed(seed)

    # import from csv
    cp = Utility.loadCoverageProblem("data/SDUAreaCoverage/OdenseSO.json", nr=3)

    # stage the experiment
    for i in range(1):
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)
        exp.solve()


def findDatasetFiles(dataset, route_file, directory="data/AreaCoverage-dataset/"):
    result = []
    for filename in os.listdir(directory + dataset):
        filepath = os.path.join(directory + dataset, filename)
        file_names = []
        if os.path.isdir(filepath):
            file_names = Utility.loadDataset(filepath, route_file, "holes", "outer_polygon")
        result.append(file_names)
    return result


def saveCoverageProblem():
    # TODO save the coverage problem dict as a json, include the routes
    pass


if __name__ == "__main__":
    seed = 135239
    np.random.seed(seed)
    # loadH2Dataset()
    files = findDatasetFiles("AC300", "mem_inf_route_data0")
    # files = findDatasetFiles("H2", "mem_mlc_route_data0")
    # files = findDatasetFiles("RAL_main", "mem_mlc_route_data0")
    # files = findDatasetFiles("VM25", "mem_r1_route_data0")
    for file_names in files:
        task_dict = Utility.loadRoutePlan(file_names[0])
        if len(task_dict["lines"]) > 50:
            continue
        print("Number of tasks", len(task_dict["lines"]))
        # For each dataset convert it to a json, save it and load it to a coverage problem
        holes_file = "" if len(file_names) < 3 else file_names[2]
        polygon_dict = Utility.loadPolygonFile(file_names[1], holes_file)
        combined_dict = {**task_dict, **polygon_dict}
        cp = Utility.loadCoverageProblemFromDict(combined_dict, 3)
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=True, max_iterations=200)
        exp.solve(profiling_enabled=False)
        totalRouteLength, sumOfTaskLengths, iterations, computeTime = exp.evaluateSolution()
