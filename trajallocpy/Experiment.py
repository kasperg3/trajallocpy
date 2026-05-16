import copy
import json
import multiprocessing
import threading
import timeit
from multiprocessing import Queue
from threading import Thread

import numpy as np
import shapely

from trajallocpy import ACBBA, CBBA, PI, Agent, CoverageProblem, Utility


class Runner:
    def __init__(
        self,
        coverage_problem: CoverageProblem.CoverageProblem,
        agents: list[Agent.config],
        enable_plotting=False,
        algorithm: str = "CBBA",
    ):
        # Task definition
        self.coverage_problem = coverage_problem
        self.robot_list = {}
        self.algorithm = algorithm
        agent_cls = PI.agent if algorithm == "PI" else CBBA.agent

        for agent in agents:
            self.robot_list[agent.id] = agent_cls(
                id=agent.id,
                state=shapely.Point(agent.position),
                environment=copy.deepcopy(self.coverage_problem.environment),
                tasks=np.array(self.coverage_problem.getTasks()),
                capacity=agent.capacity,
                number_of_agents=len(agents),
                point_estimation=False,
            )
        self.communication_graph = np.ones((len(agents), len(agents)))
        self.plot = enable_plotting

        # Results
        self.routes = {}
        self.transport = {}
        self.tasks = {}

    def evaluateSolution(self, show=True):
        path_lengths = {}
        task_length = {}
        path_costs = {}
        route_list = {}
        rewards = {}
        for r in self.robot_list.values():
            r: CBBA.agent
            path_lengths[r.id] = Agent.getTotalPathLength(r.state, r.getPathTasks(), r.environment)
            task_length[r.id] = Agent.getTotalTaskLength(r.getPathTasks())
            path_costs[r.id] = Agent.getTotalTravelCost(r.state, r.getPathTasks(), r.environment)
            rewards[r.id] = Agent.calculatePathReward(r.state, r.getPathTasks(), r.environment, r.capacity, r.Lambda)

            route = [r.state]
            for task in r.getPathTasks():
                route.extend(list(task.trajectory.coords))
            route.append(r.state)
            route_list[r.id] = route
        self.time_window_violations = sum(
            Agent.countTimeWindowViolations(r.state, r.getPathTasks(), r.environment) for r in self.robot_list.values()
        )
        self.allocated_tasks = sum(len(r.path) for r in self.robot_list.values())
        print("Results")
        print("Execution time: ", self.end_time - self.start_time)
        print("Iterations: ", self.iterations)
        print("Path lengths: ", path_lengths)
        print("Task lengths: ", task_length)
        print("Path costs: ", path_costs)
        print("Rewards: ", rewards)
        # print("Routes: ", route_list)
        max_path_cost = max(path_costs.values())
        print("Max path cost: ", max_path_cost)
        print("Sum of path lengths: ", sum(path_lengths.values()))
        print("Sum of task lengths: ", sum(task_length.values()))
        print("Sum of path costs: ", sum(path_costs.values()))
        print("Sum of rewards: ", sum(rewards.values()))
        print("Time window violations: ", self.time_window_violations)
        return (
            self.end_time - self.start_time,
            self.iterations,
            path_lengths,
            task_length,
            path_costs,
            rewards,
            route_list,
            max_path_cost,
        )

    def add_tasks(self, tasks):
        # TODO make sure that the tasks are within the search area

        # TODO make sure that the tasks not already in the list
        for robot in self.robot_list:
            robot.add_tasks(tasks)

    def solve(self, profiling_enabled=False, debug=False):
        t = 0  # Iteration number

        self.start_time = timeit.default_timer()

        result_queue = multiprocessing.Queue()
        # result_queue.cancel_join_thread()
        use_threads = True
        number_of_messages = 0

        while True:
            print("Iteration {}".format(t + 1))
            # Phase 1: Auction Process

            # Create a list to store the threads
            processes: list[multiprocessing.Process] = []
            # Start multiple threads
            if use_threads:
                for robot in self.robot_list.values():
                    # robot.build_bundle(result_queue)
                    process = multiprocessing.Process(target=robot.build_bundle, args=(result_queue,))
                    process.start()
                    processes.append(process)

                # Wait for all processes to finish
                for process in processes:
                    process.join()

                # Extract results from the queue
                while not result_queue.empty():
                    result = result_queue.get()
                    self.robot_list[result.id].update_bundle_result(result)
            else:  # Single thread
                for robot in self.robot_list.values():
                    robot.build_bundle()
            if debug:
                print("Bundle")
                for robot in self.robot_list.values():
                    print(robot.bundle)
                print("Path")
                for robot in self.robot_list.values():
                    print(robot.path)

            # Do not communicate if there are no agents to communicate with
            if len(self.robot_list) <= 1:
                break

            # Communication stage
            message_pool = [robot.send_message() for robot in self.robot_list.values()]
            conflicts = 0
            # Phase 2: Consensus Process
            if isinstance(self.robot_list[0], ACBBA.agent):  # ACBBA
                messages = 0
                for robot in self.robot_list.values():
                    robot: ACBBA.Agent
                    # Update local information and decision
                    conflicts += len(robot.update_task(robot.Y))

                if messages == 0:
                    break
            else:  # CBBA
                for robot_id, robot in self.robot_list.items():
                    robot: CBBA.agent
                    # Recieve winning bidlist from neighbors
                    g = self.communication_graph[robot_id]

                    (connected,) = np.where(g == 1)
                    connected = list(connected)
                    connected.remove(robot_id)
                    conflicts += robot.update_task({neighbor_id: message_pool[neighbor_id] for neighbor_id in connected})
                if debug:
                    print("Conflicts:", conflicts)

            if conflicts == 0:
                print("Converged in {} iterations, and sent {} messages".format(t + 1, number_of_messages))
                break

            if debug:
                print("Bundle")
                for robot in self.robot_list.values():
                    print(robot.bundle)
                print("Path")
                for robot in self.robot_list.values():
                    print(robot.path)

            t += 1

        self.iterations = t

        self.end_time = timeit.default_timer()

        # Save the results in the object
        for robot in self.robot_list.values():
            self.routes[robot.id], self.transport[robot.id], self.tasks[robot.id] = Agent.getTravelPath(
                robot.state, robot.getPathTasks(), robot.environment
            )

        # Export the transport and tasks to a JSON file
        with open("transport.json", "w") as json_file:
            json.dump(self.transport, json_file, default=lambda o: o.__dict__, indent=4)
        with open("tasks.json", "w") as json_file:
            json.dump(self.tasks, json_file, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else o.__dict__, indent=4)
        with open("routes.json", "w") as json_file:
            json.dump(self.routes, json_file, default=lambda o: o.__dict__, indent=4)
        # Print the agent bundles
        for robot_id, robot in self.robot_list.items():
            print(f"Agent {robot_id} bundle: {robot.bundle}")
