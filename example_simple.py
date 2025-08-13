#!/usr/bin/env python3
"""
Simple Example: Multi-Agent Task Allocation with TrajAllocPy

This example demonstrates how to use TrajAllocPy for multi-agent trajectory
task allocation using the Consensus Based Bundle Algorithm (CBBA).
"""

import geojson
import numpy as np
import shapely
from shapely import geometry

from trajallocpy import Agent, CoverageProblem, Experiment, Task


def create_simple_environment():
    """
    Create a simple rectangular environment with some obstacles and tasks.

    Returns:
        dict: Dictionary containing boundary, obstacles, and tasks geometries
    """
    # Define a simple rectangular boundary (100x100 units)
    boundary = shapely.geometry.Polygon([
        [0, 0], [100, 0], [100, 100], [0, 100], [0, 0]
    ])

    # Create some simple obstacles (rectangles)
    obstacle1 = shapely.geometry.Polygon([
        [20, 20], [40, 20], [40, 40], [20, 40], [20, 20]
    ])
    obstacle2 = shapely.geometry.Polygon([
        [60, 60], [80, 60], [80, 80], [60, 80], [60, 60]
    ])
    obstacles = shapely.geometry.MultiPolygon([obstacle1, obstacle2])

    # Create some simple tasks (line segments to cover)
    task1 = shapely.geometry.LineString([(10, 10), (15, 15)])
    task2 = shapely.geometry.LineString([(50, 30), (70, 35)])
    task3 = shapely.geometry.LineString([(30, 70), (45, 85)])
    task4 = shapely.geometry.LineString([(85, 15), (90, 25)])
    tasks = shapely.geometry.MultiLineString([task1, task2, task3, task4])

    return {
        "boundary": boundary,
        "obstacles": obstacles,
        "tasks": tasks
    }


def run_simple_example():
    """
    Run a simple multi-agent task allocation example.
    """
    print("=== TrajAllocPy Simple Example ===")
    
    # Create the environment
    geometries = create_simple_environment()
    
    # Configuration parameters
    n_agents = 3
    agent_capacity = 500  # Maximum distance each agent can travel
    
    print(f"Environment: {len(list(geometries['tasks'].geoms))} tasks")
    print(f"Agents: {n_agents} agents with capacity {agent_capacity}")
    
    # Create task list with rewards
    task_list = []
    for task_id, task_geom in enumerate(geometries["tasks"].geoms):
        # Each task has a reward of 100 points
        task_list.append(Task.TrajectoryTask(task_id, task_geom, reward=100))
    
    # Initialize the coverage problem
    coverage_problem = CoverageProblem.CoverageProblem(
        restricted_areas=geometries["obstacles"],
        search_area=geometries["boundary"],
        tasks=task_list,
    )
    
    # Create agents with random starting positions
    agent_list = []
    for agent_id in range(n_agents):
        start_position = coverage_problem.generate_random_point_in_problem().coords.xy
        agent = Agent.config(
            agent_id,
            start_position,
            agent_capacity,
            max_velocity=10
        )
        agent_list.append(agent)
        print(f"Agent {agent_id} starting at: ({start_position[0][0]:.1f}, {start_position[1][0]:.1f})")

    # Set up and run the experiment
    experiment = Experiment.Runner(
        coverage_problem=coverage_problem,
        enable_plotting=True,  # Set to False to disable visualization
        agents=agent_list
    )
    
    print("\nSolving task allocation...")
    experiment.solve(profiling_enabled=False, debug=False)
    
    # Evaluate and display results
    (
        compute_time,
        iterations,
        path_lengths_dict,
        task_lengths_dict,
        path_costs_dict,
        rewards_dict,
        route_list,
        max_route_cost,
    ) = experiment.evaluateSolution()

    print("\n=== Results ===")
    print(f"Computation time: {compute_time:.3f} seconds")
    print(f"Iterations: {iterations}")
    print(f"Total route length: {sum(path_lengths_dict.values()):.2f}")
    print(f"Total route costs: {sum(path_costs_dict.values()):.2f}")
    print(f"Maximum route cost: {max_route_cost:.2f}")
    print(f"Total rewards: {sum(rewards_dict.values()):.2f}")

    print("\nTask allocation per agent:")
    for agent_id in rewards_dict:
        print(f"Agent {agent_id}: Route length {path_lengths_dict[agent_id]:.1f}, "
              f"Reward {rewards_dict[agent_id]:.1f}")
def load_from_geojson_file(filename):
    """
    Load environment from a GeoJSON file (like the provided environment files).

    Args:
        filename (str): Path to the GeoJSON file

    Returns:
        dict: Dictionary containing boundary, obstacles, and tasks geometries
    """
    with open(filename) as json_file:
        geojson_file = geojson.load(json_file)
        features = geojson_file["features"]

    geometries = {
        "obstacles": shapely.MultiPolygon(),
        "tasks": shapely.MultiLineString(),
        "boundary": shapely.Polygon(),
    }

    for feature in features:
        if feature["geometry"]:
            geometries[feature["id"]] = geometry.shape(feature["geometry"])

    return geometries


def run_geojson_example():
    """
    Run an example using the provided GeoJSON environment file.
    """
    print("\n=== TrajAllocPy GeoJSON Example ===")
    
    # Load from the provided environment file
    try:
        geometries = load_from_geojson_file("environment_heatmap_coverage.geojson")
        
        # Configuration
        n_agents = 3
        agent_capacity = 2000
        
        number_of_tasks = len(list(geometries["tasks"].geoms))
        print(f"Loaded environment with {number_of_tasks} tasks")
        
        # Create task list
        task_list = []
        for task_id, task_geom in enumerate(geometries["tasks"].geoms):
            task_list.append(Task.TrajectoryTask(task_id, task_geom, reward=100))
        
        # Buffer obstacles to create safe distance
        scaled_polygons = []
        for polygon in geometries["obstacles"].geoms:
            scaled_polygon = polygon.buffer(-1)
            scaled_polygons.append(scaled_polygon)
        scaled_obstacles = shapely.geometry.MultiPolygon(scaled_polygons)
        
        # Initialize coverage problem
        coverage_problem = CoverageProblem.CoverageProblem(
            restricted_areas=scaled_obstacles,
            search_area=geometries["boundary"].buffer(1),
            tasks=task_list,
        )
        
        # Create agents
        agent_list = []
        for agent_id in range(n_agents):
            start_pos = coverage_problem.generate_random_point_in_problem().coords.xy
            agent_list.append(Agent.config(agent_id, start_pos, agent_capacity, max_velocity=10))
        
        # Run experiment
        experiment = Experiment.Runner(
            coverage_problem=coverage_problem,
            enable_plotting=True,
            agents=agent_list
        )

        experiment.solve(profiling_enabled=False, debug=False)
        
        # Show results
        results = experiment.evaluateSolution()
        compute_time, path_lengths_dict = results[0], results[2]
        print(f"Computation time: {compute_time:.3f} seconds")
        print(f"Total route length: {sum(path_lengths_dict.values()):.2f}")
        print(f"Tasks in environment: {len(task_list)}")
        
    except FileNotFoundError:
        print("GeoJSON file not found. Run the simple example instead.")
        return False
    
    return True


if __name__ == "__main__":
    # Set random seed for reproducible results
    np.random.seed(123)
    
    # Run the simple example
    run_simple_example()
    
    # Optionally run the GeoJSON example if file exists
    print("\n" + "="*50)
    if not run_geojson_example():
        print("Skipping GeoJSON example - file not available")
    
    print("\nExample completed!")
