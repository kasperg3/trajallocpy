import copy
import json
import multiprocessing
import os
import timeit
from dataclasses import dataclass

import numpy as np
import shapely

from trajallocpy import ACBBA, CBBA, PI, Agent, CoverageProblem, Transport
from trajallocpy._logging import logger


@dataclass
class AllocationResult:
    """Outcome of a solve.

    Iterates as the historical 8-tuple
    ``(compute_time, iterations, path_lengths, task_lengths, path_costs,
    rewards, routes, max_path_cost)`` so existing ``a, b, ... = result``
    unpacking keeps working; new code should use the named fields.
    """

    compute_time: float
    iterations: int
    path_lengths: dict
    task_lengths: dict
    path_costs: dict
    rewards: dict
    routes: dict
    max_path_cost: float
    time_window_violations: int = 0
    allocated_tasks: int = 0
    converged: bool = True

    def __iter__(self):
        yield from (
            self.compute_time,
            self.iterations,
            self.path_lengths,
            self.task_lengths,
            self.path_costs,
            self.rewards,
            self.routes,
            self.max_path_cost,
        )


def _build_graph(communication_graph, n):
    if communication_graph is None:
        return Transport.CommunicationGraph.full(n)
    if isinstance(communication_graph, Transport.CommunicationGraph):
        return communication_graph
    return Transport.CommunicationGraph.from_matrix(np.asarray(communication_graph))


class Runner:
    def __init__(
        self,
        coverage_problem: CoverageProblem.CoverageProblem,
        agents: list,
        enable_plotting=False,
        algorithm: str = "CBBA",
        communication_graph=None,
        link_model=None,
        seed=None,
        export_dir=None,
        max_runtime: float = 30.0,
        async_mode: str = "step",
    ):
        self.coverage_problem = coverage_problem
        self.robot_list = {}
        self.algorithm = algorithm
        self.export_dir = export_dir
        self.max_runtime = max_runtime
        self.async_mode = async_mode
        self.converged = True

        if algorithm == "PI":
            agent_cls = PI.agent
        elif algorithm == "ACBBA":
            agent_cls = ACBBA.agent
        else:
            agent_cls = CBBA.agent

        # The simulated network only matters for the asynchronous ACBBA path.
        if algorithm == "ACBBA" and async_mode == "step":
            clock = Transport.LogicalClock()
        else:
            import time

            clock = time.monotonic
        graph = _build_graph(communication_graph, len(agents))
        self.transport_layer = Transport.Transport(graph, link=link_model, seed=seed, clock=clock)

        for agent in agents:
            common = {
                "id": agent.id,
                "state": shapely.Point(agent.position),
                "environment": copy.deepcopy(self.coverage_problem.environment),
                "tasks": np.array(self.coverage_problem.getTasks()),
                "capacity": agent.capacity,
                "point_estimation": False,
                "max_velocity": agent.max_velocity,
                "max_acceleration": agent.max_acceleration,
                "Lambda": agent.Lambda,
                "removal_threshold": agent.removal_threshold,
            }
            if algorithm == "ACBBA":
                self.robot_list[agent.id] = agent_cls(clock=clock, **common)
            else:
                self.robot_list[agent.id] = agent_cls(number_of_agents=len(agents), **common)

        # Kept for the synchronous CBBA/PI path (full connectivity assumption).
        self.communication_graph = np.ones((len(agents), len(agents)))
        self.enable_plotting = enable_plotting

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
        max_path_cost = max(path_costs.values()) if path_costs else 0.0
        if show:
            logger.info("Execution time: %s", self.end_time - self.start_time)
            logger.info("Iterations: %s", self.iterations)
            logger.info("Path lengths: %s", path_lengths)
            logger.info("Task lengths: %s", task_length)
            logger.info("Path costs: %s", path_costs)
            logger.info("Rewards: %s", rewards)
            logger.info("Max path cost: %s", max_path_cost)
            logger.info("Sum of path lengths: %s", sum(path_lengths.values()))
            logger.info("Sum of task lengths: %s", sum(task_length.values()))
            logger.info("Sum of path costs: %s", sum(path_costs.values()))
            logger.info("Sum of rewards: %s", sum(rewards.values()))
            logger.info("Time window violations: %s", self.time_window_violations)
        return AllocationResult(
            compute_time=self.end_time - self.start_time,
            iterations=self.iterations,
            path_lengths=path_lengths,
            task_lengths=task_length,
            path_costs=path_costs,
            rewards=rewards,
            routes=route_list,
            max_path_cost=max_path_cost,
            time_window_violations=self.time_window_violations,
            allocated_tasks=self.allocated_tasks,
            converged=self.converged,
        )

    def add_tasks(self, tasks):
        for robot in self.robot_list:
            robot.add_tasks(tasks)

    def _finalize(self):
        for robot in self.robot_list.values():
            self.routes[robot.id], self.transport[robot.id], self.tasks[robot.id] = Agent.getTravelPath(
                robot.state, robot.getPathTasks(), robot.environment
            )
        if self.export_dir is not None:
            os.makedirs(self.export_dir, exist_ok=True)
            with open(os.path.join(self.export_dir, "transport.json"), "w") as json_file:
                json.dump(self.transport, json_file, default=lambda o: o.__dict__, indent=4)
            with open(os.path.join(self.export_dir, "tasks.json"), "w") as json_file:
                json.dump(self.tasks, json_file, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else o.__dict__, indent=4)
            with open(os.path.join(self.export_dir, "routes.json"), "w") as json_file:
                json.dump(self.routes, json_file, default=lambda o: o.__dict__, indent=4)
        for robot_id, robot in self.robot_list.items():
            logger.debug("Agent %s bundle: %s", robot_id, robot.bundle)

    def solve(self, profiling_enabled=False, debug=False):
        if self.algorithm == "ACBBA":
            from trajallocpy import AsyncRunner

            AsyncRunner.run(self, mode=self.async_mode, max_runtime=self.max_runtime)
            self._finalize()
            return

        t = 0  # Iteration number
        self.start_time = timeit.default_timer()
        result_queue = multiprocessing.Queue()
        use_threads = True

        while True:
            logger.debug("Iteration %s", t + 1)
            processes: list = []
            if use_threads:
                for robot in self.robot_list.values():
                    process = multiprocessing.Process(target=robot.build_bundle, args=(result_queue,))
                    process.start()
                    processes.append(process)
                for process in processes:
                    process.join()
                while not result_queue.empty():
                    result = result_queue.get()
                    self.robot_list[result.id].update_bundle_result(result)
            else:
                for robot in self.robot_list.values():
                    robot.build_bundle()
            if debug:
                for robot in self.robot_list.values():
                    logger.debug("bundle %s path %s", robot.bundle, robot.path)

            if len(self.robot_list) <= 1:
                break

            message_pool = [robot.send_message() for robot in self.robot_list.values()]
            conflicts = 0
            for robot_id, robot in self.robot_list.items():
                g = self.communication_graph[robot_id]
                (connected,) = np.where(g == 1)
                connected = list(connected)
                connected.remove(robot_id)
                conflicts += robot.update_task({neighbor_id: message_pool[neighbor_id] for neighbor_id in connected})
            if debug:
                logger.debug("Conflicts: %s", conflicts)

            if conflicts == 0:
                logger.debug("Converged in %s iterations", t + 1)
                break
            t += 1

        self.iterations = t
        self.end_time = timeit.default_timer()
        self._finalize()

    def plot(self, save_path=None, block=False):
        """Render the current allocation. matplotlib is imported lazily so the
        library stays headless-friendly; if ``save_path`` is given the figure
        is written there instead of shown."""
        from trajallocpy import Utility

        plotter = Utility.Plotter(list(self.robot_list.values()), self.communication_graph)
        search_area = self.coverage_problem.getSearchArea()
        if search_area is not None:
            plotter.plotPolygon(search_area, "lightgrey", fill=False)
        restricted = self.coverage_problem.getRestrictedAreas()
        if restricted is not None:
            plotter.plotMultiPolygon(restricted, "red", fill=True)
        plotter.plotAgents(list(self.robot_list.values()))
        if save_path is not None:
            plotter.save(save_path)
        else:
            import matplotlib.pyplot as plt

            plt.show(block=block)
        return plotter
