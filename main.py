import json
from task_allocation import Experiment, Utility
import numpy as np
import os
import csv
import argparse


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


def saveResults(experiment_title, results):
    results_header = [
        "dataset_name",
        "totalRouteLength",
        "sumOfTaskLengths",
        "iterations",
        "computeTime",
        "num_tasks",
        "number_of_agents",
    ]
    with open("benchmarks/" + experiment_title + ".csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(results_header)
        writer.writerows(results)


def main(dataset_name, route_filename, experiment_title, number_of_agents):
    seed = 135239
    np.random.seed(seed)

    results = []
    files = findDatasetFiles(dataset_name, route_filename)
    for file_names in files:
        task_dict = Utility.loadRoutePlan(file_names[0])
        if len(task_dict["lines"]) > 100:
            continue
        print("Number of tasks", len(task_dict["lines"]))
        # For each dataset convert it to a json, save it and load it to a coverage problem
        holes_file = "" if len(file_names) < 3 else file_names[2]
        polygon_dict = Utility.loadPolygonFile(file_names[1], holes_file)
        combined_dict = {**task_dict, **polygon_dict}
        cp = Utility.loadCoverageProblemFromDict(combined_dict, number_of_agents)
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=False, max_iterations=200)
        exp.solve(profiling_enabled=False)

        # Save the results in a csv file
        totalRouteLength, sumOfTaskLengths, iterations, computeTime = exp.evaluateSolution()
        results.append(
            [
                file_names[1].replace("/outer_polygon", ""),
                totalRouteLength,
                sumOfTaskLengths,
                iterations,
                computeTime,
                len(task_dict["lines"]),
                number_of_agents,
            ]
        )
        saveResults(experiment_title, results)

        # Save the route files


if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser(description="Calculates a conflict free task allocation")
    parser.add_argument("--dataset", type=str, help="The name of the dataset")
    parser.add_argument("--route_file_name", type=str, help="the filename of the line file")
    parser.add_argument("--experiment_name", type=str, help="The name of the experiment")
    parser.add_argument("--n_robots", type=int, help="The number of robots to include")
    args = parser.parse_args()

    main(args.dataset, args.route_file_name, args.experiment_name, args.n_robots)
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_3robots_AC300 --n_robots=3
    # experiment_title = "ours_3robots_AC300"
    # main("AC300", "mem_inf_route_data0", experiment_title, 3)
    # main("H2", "mem_mlc_route_data0", 3)
    # main("RAL_main", "mem_mlc_route_data0", 3)
    # main("VM25", "mem_r1_route_data0", 3)
