import argparse
import csv
import os
import sys

import geojson
import numpy as np

from task_allocation import CoverageProblem, Experiment, Utility


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
    number_of_agents,
    capacity=None,
    point_estimation=False,
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

        cp = CoverageProblem.CoverageProblem(features, number_of_robots=number_of_agents)
        exp = Experiment.runner(
            coverage_problem=cp,
            enable_plotting=show_plots,
            max_iterations=200,
            task_capacity=capacity,
            use_point_estimation=point_estimation,
        )
        if show_plots:
            Utility.plotGraph(cp.travel_graph, cp.getSearchArea(), cp.getRestrictedAreas(), cp.getTasks())
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
                file_name,
                totalRouteLength,
                sumOfTaskLengths,
                totalRouteCosts,
                maxRouteCost,
                iterations,
                computeTime,
                cp.getNumberOfTasks(),
                number_of_agents,
            ]
        )

        # TODO in Experiment, create a scenario builder, where the mission or experiment can be conducted:
        # It should be able to replan the mission at different point of time.
        # The algorithm should be aware of which missions have already been executed and not being able to bid on these.

        # Find a way of "playing" a scenario and doing replanning and removing/adding tasks dynamically
        # Read litterature
        # TODO:
        # Scenariobuilder
        # Ability to jump to a point in time
        # remove/add tasks/agents at a certain time
        # evaluate the performance by spawning a survivor at location which a single task will cover
        # The survivor/survivors should be randomly sampled

        # What happens when an agent leaves the group:
        # * unallocate not finished tasks

        # Save the results to the csv
        saveResults(experiment_title, results)


if __name__ == "__main__":
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
            number_of_agents=n_agents,
            capacity=capacity,
            point_estimation=False,
            show_plots=True,
            debug=False,
        )
