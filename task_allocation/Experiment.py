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
    ):
        # Task definition
        self.coverage_problem = coverage_problem
        self.task_num = int(len(self.coverage_problem.getSweeps()))
        self.robot_num = self.coverage_problem.getNumberOfRobots()

        # TODO this should be based able to be based on distance/batterylife
        task_capacity = 10

        self.task = np.array(self.coverage_problem.getSweeps())

        # TODO do not use the first task as initial state
        initial_state = np.array(self.task[0])

        self.robot_list = [
            CBBA.agent(
                id=i,
                task_num=self.task_num,
                agent_num=self.robot_num,
                L_t=task_capacity,
                state=initial_state,
            )
            for i in range(self.robot_num)
        ]

        self.communication_graph = coverage_problem.getCommunicationGraph()
        self.max_t = max_iterations
        self.plot = enable_plotting

    def run(self):
        t = 0  # Iteration number
        plotter = Utility.Plotter(self.task, self.robot_list, self.communication_graph)

        # Plot the search area and restricted area
        plotter.plotAreas(self.coverage_problem.getSearchArea(), color=[0, 0, 0])
        plotter.plotAreas(self.coverage_problem.getRestrictedAreas(), color=[0, 0, 0])
        # TODO plot the sweep tasks

        starttime = timeit.default_timer()

        while True:
            converged_list = []

            print("Iteration {}".format(t))
            # Phase 1: Auction Process
            for robot in self.robot_list:
                robot.build_bundle(self.task)

            # Plot
            if self.plot:
                plotter.setTitle("Time Step:{}, Bundle Construct".format(t))
                for robot in self.robot_list:
                    plotter.plotAgents(robot, self.task, t)
                plotter.pause()

            # Communication stage
            # Send winning bid list to neighbors (depend on env)
            message_pool = [robot.send_message() for robot in self.robot_list]
            for robot_id, robot in enumerate(self.robot_list):
                # Recieve winning bidlist from neighbors
                g = self.communication_graph[robot_id]

                (connected,) = np.where(g == 1)
                connected = list(connected)

                Y = (
                    {
                        neighbor_id: message_pool[neighbor_id]
                        for neighbor_id in connected
                    }
                    if len(connected) > 0
                    else None
                )

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
                    plotter.plotAgents(robot, self.task, t)

                plotter.pause()
            t += 1

            if sum(converged_list) == self.robot_num or t > self.max_t:
                break

        print("Robot Routes")
        for robot in self.robot_list:
            print(robot.path)

        print("Execution time:", timeit.default_timer() - starttime)
        if self.plot:
            plotter.show()
