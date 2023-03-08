import json
from task_allocation import Experiment, Utility
import numpy as np
import os
import csv
import argparse

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
        "totalRouteCosts",
        "maxRouteCost",
        "iterations",
        "computeTime",
        "num_tasks",
        "number_of_agents",
    ]
    with open("benchmarks/" + experiment_title + ".csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(results_header)
        writer.writerows(results)

def main(
    dataset_name,
    route_filename,
    experiment_title,
    number_of_agents,
    capacity=None,
    point_estimation=False,
    show_plots=True,
    debug=False,
):
    seed = 135239
    np.random.seed(seed)

    results = []
    for file_names in files:
        cp = Utility.loadCoverageProblemFromDict(combined_dict, number_of_agents)
        exp = Experiment.runner(coverage_problem=cp, enable_plotting=show_plots,max_iterations=200,task_capacity=capacity,use_point_estimation=point_estimation)
        exp.solve(profiling_enabled=False, debug=debug)

        # Save the results in a csv file
        (totalRouteLength, sumOfTaskLengths, totalRouteCosts, iterations, computeTime, route_list, maxRouteCost) = exp.evaluateSolution()

        results.append(
            [
                file_names[1].replace("/outer_polygon", ""),
                totalRouteLength,
                sumOfTaskLengths,
                totalRouteCosts,
                maxRouteCost,
                iterations,
                computeTime,
                len(task_dict["lines"]),
                number_of_agents,
            ]
        )

        # Save the results to the csv
        saveResults(experiment_title, results)


if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser(description="Calculates a conflict free task allocation")
    parser.add_argument("--dataset", type=str, help="The name of the dataset")
    parser.add_argument("--route_file_name", type=str, help="the filename of the line file")
    parser.add_argument("--experiment_name", type=str, help="The name of the experiment")
    parser.add_argument("--n_robots", type=int, help="The number of robots to include")
    parser.add_argument("--capacity", type=int, help="The capacity of the robots given in minutes")
    parser.add_argument(
        "--point_estimation",
        default=False,
        type=bool,
        help="Bool for wether to use point estimation",
    )
    parser.add_argument("--show_plots", default=False, type=bool, help="whether to show plots")
    args = parser.parse_args()
    print(args)
    if(len(args._get_args()) != 0):
        main(
            args.dataset,
            args.route_file_name,
            args.experiment_name,
            args.n_robots,
            args.capacity,
            args.point_estimation,
            args.show_plots,
        )
    else: 
        experiment_title = "AC300_convergence_14robots_test"
        main( dataset_name="AC300", route_filename="mem_inf_route_data0", experiment_title=experiment_title, number_of_agents=14, capacity=300, point_estimation=False, show_plots=False, debug=False,)
