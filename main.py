import argparse
import csv
import os
import random
import sys

import geojson
import numpy as np
import shapely
from shapely import geometry
from shapely.affinity import scale, translate
from shapely.ops import transform

from trajallocpy import Agent, CoverageProblem, Experiment, Task, Utility


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
    results = []
    run_experiment(
        "custom",
        n_agents,
        capacity,
        show_plots,
        debug,
        results,
        "environment.geojson",
    )

    # files = Utility.getAllCoverageFiles(dataset_name)
    # for file_name in files:
    #     run_experiment(experiment_title, n_agents, capacity, show_plots, debug, results, file_name)


def run_experiment(experiment_title, n_agents, capacity, show_plots, debug, results, file_name, export=True):
    with open(file_name) as json_file:
        geojson_file = geojson.load(json_file)
        try:
            geojson_file["crs"]
        except KeyError:
            print("Warning! No CRS is given and can cause odd behaviours!")
        features = geojson_file["features"]

    geometries = {
        "obstacles": shapely.MultiPolygon(),
        "tasks": shapely.MultiLineString(),
        "boundary": shapely.Polygon(),
    }

    for feature in features:
        if feature["geometry"]:
            geometries[feature["id"]] = geometry.shape(feature["geometry"])
    number_of_tasks = len(list(geometries["tasks"].geoms))

    # Normalize the geoms
    min_x, min_y, _, _ = geometries["boundary"].bounds
    for key in geometries:
        geometries[key] = translate(geometries[key], -min_x, -min_y)

    print(file_name, " Tasks: ", number_of_tasks)
    # Initialize coverage problem and the agents

    geometries["boundary"] = geometries["boundary"].buffer(1)

    # Scale each polygon in the MultiPolygon
    scaled_polygons = []
    for polygon in geometries["obstacles"].geoms:
        scaled_polygon = polygon.buffer(-1)  # scale(polygon, xfact=0.95, yfact=0.95, origin="centroid")
        scaled_polygons.append(scaled_polygon)

        # Create a new MultiPolygon with scaled polygons
    scaled_multi_polygon = shapely.geometry.MultiPolygon(scaled_polygons)
    task_list = []
    for id, task in enumerate(geometries["tasks"].geoms):
        task_list.append(Task.TrajectoryTask(id, task))

    cp = CoverageProblem.CoverageProblem(
        restricted_areas=scaled_multi_polygon,
        search_area=geometries["boundary"],
        tasks=task_list,
    )

    initial = cp.generate_random_point_in_problem().coords.xy
    agent_list = [
        Agent.config(id, (initial[0][0] + random.uniform(-10, 10), initial[1][0] + random.uniform(-10, 10)), capacity, max_velocity=10)
        for id in range(n_agents)
    ]
    exp = Experiment.Runner(coverage_problem=cp, enable_plotting=show_plots, agents=agent_list)

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
            len(agent_list),
        ]
    )

    # Save the results to the csv
    saveResults(experiment_title, results)


if __name__ == "__main__":
    seed = 123
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
        n_agents = 3
        capacity = 5000
        main(
            dataset_name=ds,
            experiment_title=ds + "_" + str(n_agents) + "agents_" + str(capacity) + "capacity",
            n_agents=n_agents,
            capacity=capacity,
            show_plots=True,
            debug=False,
        )
