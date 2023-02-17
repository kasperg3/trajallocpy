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


def loadAC300Dataset():
    directory = "data/AreaCoverage-dataset/"
    dataset = "AC300"
    for filename in os.listdir(directory + dataset):
        filepath = os.path.join(directory + dataset, filename)
        # if filepath.endswith("AC1_0016"):
        # Check if it is a directory
        file_names = []
        if os.path.isdir(filepath):
            file_names = Utility.loadDataset(
                filepath, "mem_inf_route_data0", "holes", "outer_polygon"
            )

        task_dict = Utility.loadRoutePlan(file_names[0])
        # if len(task_dict["lines"]) > 60:
        #     continue
        print(len(task_dict["lines"]))
        # For each dataset convert it to a json, save it and load it to a coverage problem
        polygon_dict = Utility.loadPolygonFile(file_names[1], file_names[2])
        combined_dict = {**task_dict, **polygon_dict}
        cp = Utility.loadCoverageProblemFromDict(combined_dict, 3)
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=False)
        exp.solve(profiling_enabled=False)


def loadH2Dataset():
    directory = "data/AreaCoverage-dataset/"
    dataset = "H2"
    for filename in os.listdir(directory + dataset):
        filepath = os.path.join(directory + dataset, filename)
        # if filepath.endswith("AC1_0016"):
        # Check if it is a directory
        file_names = []
        if os.path.isdir(filepath):
            file_names = Utility.loadDataset(
                filepath, "mem_mlc_route_data0", "holes", "outer_polygon"
            )

        tasks_file = file_names[0]
        polygon_file = file_names[1]
        holes_file = "" if len(file_names) < 3 else file_names[2]
        task_dict = Utility.loadRoutePlan(tasks_file)

        # For each dataset convert it to a json, save it and load it to a coverage problem
        polygon_dict = Utility.loadPolygonFile(polygon_file, holes_file)
        combined_dict = {**task_dict, **polygon_dict}
        cp = Utility.loadCoverageProblemFromDict(combined_dict, 3)
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=False)
        exp.solve(profiling_enabled=False)
        # TODO save the results and store them in a dict


if __name__ == "__main__":
    loadH2Dataset()
    # loadAC300Dataset()
