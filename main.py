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


def exportSolution(dataset_name, problem_dict, route_list):
    pass


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
    files = findDatasetFiles(dataset_name, route_filename)
    for file_names in files:
        task_dict = Utility.loadRoutePlan(file_names[2])
        print("Number of tasks", len(task_dict["lines"]))
        # For each dataset convert it to a json, save it and load it to a coverage problem
        holes_file = "" if len(file_names) < 3 else file_names[1]
        polygon_dict = Utility.loadPolygonFile(file_names[0], holes_file)
        combined_dict = {**task_dict, **polygon_dict}
        cp = Utility.loadCoverageProblemFromDict(combined_dict, number_of_agents)
        exp = Experiment.runner(
            coverage_problem=cp,
            enable_plotting=show_plots,
            max_iterations=200,
            task_capacity=capacity,
            use_point_estimation=point_estimation,
        )
        exp.solve(profiling_enabled=False, debug=debug)

        # Save the results in a csv file
        (
            totalRouteLength,
            sumOfTaskLengths,
            totalRouteCosts,
            iterations,
            computeTime,
            route_list,
            maxRouteCost,
        ) = exp.evaluateSolution()

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
    # print(args)
    # main(
    #     args.dataset,
    #     args.route_file_name,
    #     args.experiment_name,
    #     args.n_robots,
    #     args.capacity,
    #     args.point_estimation,
    #     args.show_plots,
    # )

    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_4robots_600capacity_AC300 --n_robots=4 --capacity=600
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_4robots_600capacity_AC300 --n_robots=4 --capacity=600 --point_estimation=True
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_2robots_1200capacity_AC300 --n_robots=2 --capacity=1200
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_2robots_1200capacity_AC300 --n_robots=2 --capacity=1200 --point_estimation=True
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_1robots_infcapacity_AC300 --n_robots=1
    # python main.py --dataset=AC300 --route_file_name=mem_inf_route_data0 --experiment_name=ours_naive_1robots_infcapacity_AC300 --n_robots=1 --point_estimation=True
    experiment_title = "AC300_convergence_14robots_test"
    main(
        dataset_name="AC300",
        route_filename="mem_inf_route_data0",
        experiment_title=experiment_title,
        number_of_agents=6,
        capacity=300,
        point_estimation=False,
        show_plots=False,
        debug=False,
    )
