import argparse
import csv
import os
import random
import sys

import geojson
import numpy as np
import shapely
from shapely import geometry
from shapely.affinity import scale
from shapely.ops import transform

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
    n_agents,
    capacity,
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
                geometries[feature["id"]] = geometry.shape(feature["geometry"])
        number_of_tasks = len(list(geometries["tasks"].geoms))

        print(file_name, " Tasks: ", number_of_tasks)
        # if number_of_tasks > 250:
        #     continue
        # Initialize coverage problem and the agents
        geometries["boundary"] = scale(geometries["boundary"], xfact=1.01, yfact=1.01)

        # Scale each polygon in the MultiPolygon
        scaled_polygons = []
        for polygon in geometries["obstacles"].geoms:
            scaled_polygon = scale(polygon, xfact=0.95, yfact=0.95, origin="centroid")
            scaled_polygons.append(scaled_polygon)

        # Create a new MultiPolygon with scaled polygons
        scaled_multi_polygon = shapely.geometry.MultiPolygon(scaled_polygons)

        cp = CoverageProblem.CoverageProblem(restricted_areas=scaled_multi_polygon, search_area=geometries["boundary"], tasks=geometries["tasks"])
        initial = cp.generate_random_point_in_problem().coords.xy
        agent_list = [Agent.agent(id, initial, capacity) for id in range(n_agents)]
        exp = Experiment.Runner(coverage_problem=cp, enable_plotting=show_plots, agents=agent_list)

        allocations = exp.solve(profiling_enabled=False, debug=debug)

        # replanning
        # exp.add_time(100)
        # exp.replan()

        # TODO find a way of adding an agent based on id instead of index in a list
        # This has some more work to it as this requires the index querying in cbba to be done based on hash indexing
        # exp.add_agent(Agent.agent(id=))

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
    seed = 1352323
    np.random.seed(seed)
    random.seed(seed)
    parser = argparse.ArgumentParser(description="Calculates a conflict free task allocation")
    parser.add_argument("--dataset", type=str, help="The name of the dataset")
    parser.add_argument("--experiment_name", type=str, help="The name of the experiment")
    parser.add_argument("--n_robots", type=int, help="The number of robots to include")
    parser.add_argument("--capacity", type=int, help="The capacity of the robots given in minutes")
    parser.add_argument("--point_estimation", default=False, type=bool, help="Bool for wether to use point estimation")
    parser.add_argument("--show_plots", default=False, type=bool, help="whether to show plots")
    args = parser.parse_args()
    if len(sys.argv) > 1:
        main(
            dataset_name=args.dataset,
            experiment_title=args.experiment_name,
            n_agents=args.n_robots,
            capacity=args.capacity,
            show_plots=args.show_plots,
        )
    else:
        ds = "AC300"
        n_agents = 2
        capacity = 1500

        main(
            dataset_name=ds,
            experiment_title=ds + "_" + str(n_agents) + "agents_" + str(capacity) + "capacity",
            n_agents=n_agents,
            capacity=capacity,
            show_plots=False,
            debug=False,
        )
