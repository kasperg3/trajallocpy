# Implement a Experiment class that Should
#     Initialize the solver
#     create events/scenario building
#     Parametrized so that one can test different agent number/task number
from task_allocation import CoverageProblem, CBBA, Utility
import numpy as np
import timeit
import logging as log
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
        task_capacity = self.task_num  # self.task_num

        # TODO do not use the first task as initial state
        # TODO randomly distribute the agents within the allowed area
        min_easting = 100000000
        min_northing = 100000000
        max_easting = 0
        max_northing = 0
        for t in self.tasks:
            temp = min([t.end[0], t.start[0]])
            if min_easting > temp:
                min_easting = temp
            temp = min([t.end[1], t.start[1]])
            if min_northing > temp:
                min_northing = temp
            temp = max([t.end[0], t.start[0]])
            if max_easting < temp:
                max_easting = temp
            temp = max([t.end[1], t.start[1]])
            if max_northing < temp:
                max_northing = temp

        if agents is None:
            self.robot_list = []
            for i in range(self.robot_num):
                initial_state = np.array(
                    [
                        np.random.uniform(min_easting, max_easting),
                        np.random.uniform(min_northing, max_northing),
                    ]
                )
                self.robot_list.append(
                    CBBA.agent(
                        id=i,
                        tasks=self.tasks,
                        agent_num=self.robot_num,
                        L_t=task_capacity,
                        state=initial_state,
                    )
                )
        else:
            self.robot_list = agents

        self.communication_graph = coverage_problem.getCommunicationGraph()
        self.max_t = max_iterations
        self.plot = enable_plotting

    def evaluateSolution(self):
        print("Execution time: ", self.end_time - self.start_time)
        average_travel_length = 0
        average_total_path_length = 0

        for r in self.robot_list:
            average_travel_length += r.getTotalPathCost()
            average_total_path_length += r.getTotalPathCost(True)

        print("Average Travel Length:", average_travel_length / len(self.robot_list))
        print("Average Total Path Length:", average_total_path_length / len(self.robot_list))

    def run(self, profiling_enabled=False):
        if profiling_enabled:
            import cProfile, pstats, io
            from pstats import SortKey

            pr = cProfile.Profile()
            pr.enable()
        t = 0  # Iteration number
        plotter = Utility.Plotter(self.tasks, self.robot_list, self.communication_graph)

        # Plot the search area and restricted area
        plotter.plotAreas([self.coverage_problem.getSearchArea()], color=(0, 0, 1, 0.2))
        plotter.plotAreas(self.coverage_problem.getRestrictedAreas(), color=(1, 0, 0, 0.2))
        self.start_time = timeit.default_timer()

        while True:
            converged_list = []

            print("Iteration {}".format(t))
            # Phase 1: Auction Process
            for robot in self.robot_list:
                robot.build_bundle()

            print("Bundle")
            for robot in self.robot_list:
                print(robot.getBundle())
            print("Path")
            for robot in self.robot_list:
                print(robot.getPath())

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
                connected.remove(robot_id)

                if len(connected) > 0:
                    Y = {neighbor_id: message_pool[neighbor_id] for neighbor_id in connected}
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

            print("Bundle")
            for robot in self.robot_list:
                print(robot.getBundle())
            print("Path")
            for robot in self.robot_list:
                print(robot.getPath())

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
            print(robot.getPath())

        self.end_time = timeit.default_timer()

        self.evaluateSolution()
        if self.plot:
            plotter.show()
