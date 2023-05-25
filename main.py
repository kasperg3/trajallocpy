import argparse
import csv
import os
import sys

import geojson
import numpy as np
import shapely

from task_allocation import Agent, CoverageProblem, Experiment, Utility


def saveResults(experiment_title, results, directory="experiments/"):
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
    isExist = os.path.exists(directory)
    # Create a new directory if it does not exist
    if not isExist:
        os.makedirs(directory)
    with open(directory + experiment_title + ".csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(results_header)
        writer.writerows(results)


def main(
    dataset_name,
    experiment_title,
    show_plots=True,
    debug=False,
):
    seed = 135239
    np.random.seed(seed)

    results = []
    files = Utility.getAllCoverageFiles(dataset_name)

    for file_name in files:
        with open(file_name) as json_file:
            features = geojson.load(json_file)["features"]

        geometries = {
            "obstacles": shapely.MultiPolygon(),
            "tasks": shapely.MultiLineString(),
            "boundary": shapely.Polygon(),
        }
        for feature in features:
            if feature["geometry"]:
                geometries[feature["id"]] = shapely.geometry.shape(feature["geometry"])

        cp = CoverageProblem.CoverageProblem(restricted_areas=geometries["obstacles"], search_area=geometries["boundary"], tasks=geometries["tasks"])

        agent_list = [
            Agent.agent(0, cp.generate_random_point_in_problem().coords.xy, 500),
            Agent.agent(1, cp.generate_random_point_in_problem().coords.xy, 500),
            Agent.agent(2, cp.generate_random_point_in_problem().coords.xy, 500),
            Agent.agent(3, cp.generate_random_point_in_problem().coords.xy, 500),
            Agent.agent(4, cp.generate_random_point_in_problem().coords.xy, 500),
        ]

        exp = Experiment.runner(coverage_problem=cp, enable_plotting=show_plots, agents=agent_list)
        if show_plots:
            Utility.plotGraph(cp.travel_graph, cp.getSearchArea(), cp.getRestrictedAreas(), cp.getTasks())
        allocations = exp.solve(profiling_enabled=False, debug=debug)
        print(allocations)
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
                file_name,
                totalRouteLength,
                sumOfTaskLengths,
                totalRouteCosts,
                maxRouteCost,
                iterations,
                computeTime,
                cp.getNumberOfTasks(),
                len(agent_list),
            ]
        )

        # Save the results to the csv
        saveResults(experiment_title, results)


if __name__ == "__main__":
    seed = 135239
    np.random.seed(seed)
    # main()
    parser = argparse.ArgumentParser(description="Calculates a conflict free task allocation")
    parser.add_argument("--dataset", type=str, help="The name of the dataset")
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
    if len(sys.argv) > 1:
        main(
            args.dataset,
            args.experiment_name,
            args.n_robots,
            args.capacity,
            args.point_estimation,
            args.show_plots,
        )
    else:
        ds = "AC300"
        n_agents = 4
        capacity = 1000
        use_point_est = False

        main(
            dataset_name=ds,
            experiment_title=ds + "_" + str(n_agents) + "agents_" + str(capacity) + "capacity",
            show_plots=True,
            debug=False,
        )
