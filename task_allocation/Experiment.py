# Implement a Experiment class that Should
#     Initialize the solver
#     create events/scenario building
#     Parametrized so that one can test different agent number/task number
from task_allocation import CoverageProblem, CBBA, Utility
import numpy as np
import timeit

import os


class runner:
    def __init__(
        self,
        coverage_problem: CoverageProblem.CoverageProblem,
        enable_plotting=False,
        max_iterations=100,
        agents=None,
    ):
        # Task definition
        self.coverage_problem = coverage_problem
        self.tasks = np.array(self.coverage_problem.getTasks())
        self.task_num = int(len(self.tasks))
        self.robot_num = self.coverage_problem.getNumberOfRobots()

        # TODO this should be based able to be based on distance/batterylife
        task_capacity = self.task_num

        # TODO do not use the first task as initial state
        initial_state = np.array(self.tasks[0].start)
        if agents is None:
            self.robot_list = [
                CBBA.agent(
                    id=i,
                    tasks=self.tasks,
                    agent_num=self.robot_num,
                    L_t=task_capacity,
                    state=initial_state,
                )
                for i in range(self.robot_num)
            ]
        else:
            self.robot_list = agents

        self.communication_graph = coverage_problem.getCommunicationGraph()
        self.max_t = max_iterations
        self.plot = enable_plotting

    def run(self, profiling_enabled=False):
        if profiling_enabled:
            import cProfile, pstats, io
            from pstats import SortKey

            pr = cProfile.Profile()
            pr.enable()
        t = 0  # Iteration number
        plotter = Utility.Plotter(self.tasks, self.robot_list, self.communication_graph)

        # Plot the search area and restricted area
        plotter.plotAreas([self.coverage_problem.getSearchArea()], color=(0, 0, 1, 0.3))
        plotter.plotAreas(
            self.coverage_problem.getRestrictedAreas(), color=(1, 0, 0, 0.3)
        )
        starttime = timeit.default_timer()

        while True:
            converged_list = []

            print("Iteration {}".format(t))
            # Phase 1: Auction Process
            for robot in self.robot_list:
                robot.build_bundle(self.tasks)

            # Plot
            if self.plot:
                plotter.setTitle("Time Step:{}, Bundle Construct".format(t))
                for robot in self.robot_list:
                    plotter.plotAgents(robot, self.tasks, t)
                plotter.pause(0.1)

            # Communication stage
            # Send winning bid list to neighbors (depend on env)
            message_pool = [robot.send_message() for robot in self.robot_list]
            for robot_id, robot in enumerate(self.robot_list):
                # Recieve winning bidlist from neighbors
                g = self.communication_graph[robot_id]

                (connected,) = np.where(g == 1)
                connected = list(connected)
                if len(connected) > 0:
                    Y = {
                        neighbor_id: message_pool[neighbor_id]
                        for neighbor_id in connected
                    }
                else:
                    Y = None

                robot.receive_message(Y)

            # Phase 2: Consensus Process
            for robot in self.robot_list:
                # Update local information and decision
                if Y is not None:
                    converged = robot.update_task()
                    converged_list.append(converged)

            # Plot
            if self.plot:
                plotter.setTitle("Time Step:{}, Consensus".format(t))
                for robot in self.robot_list:
                    plotter.plotAgents(robot, self.tasks, t)

                plotter.pause(0.1)
            t += 1

            if sum(converged_list) == self.robot_num or t > self.max_t:
                break

        if profiling_enabled:
            pr.disable()
            s = io.StringIO()
            sortby = SortKey.CUMULATIVE
            ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            ps.print_stats(100)

        print("Robot Routes")
        for robot in self.robot_list:
            print(robot.path)

        print("Execution time:", timeit.default_timer() - starttime)
        if self.plot:
            plotter.show()
