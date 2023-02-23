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
        initial_state=None,
        task_capacity=None,
        use_point_estimation=False,
    ):
        # Task definition
        self.coverage_problem = coverage_problem
        self.tasks = np.array(self.coverage_problem.getTasks())
        self.task_num = int(len(self.tasks))
        self.robot_num = self.coverage_problem.getNumberOfRobots()

        # TODO this should be based able to be based on distance/batterylife

        min_x = 100000000
        min_y = 100000000
        max_x = 0
        max_y = 0
        for t in self.tasks:
            temp = min([t.end[0], t.start[0]])
            if min_x > temp:
                min_x = temp
            temp = min([t.end[1], t.start[1]])
            if min_y > temp:
                min_y = temp
            temp = max([t.end[0], t.start[0]])
            if max_x < temp:
                max_x = temp
            temp = max([t.end[1], t.start[1]])
            if max_y < temp:
                max_y = temp

        if agents is None:
            if initial_state is None:
                initial_state = np.array(
                    [
                        np.random.uniform(min_x, max_x),
                        np.random.uniform(min_y, max_y),
                    ]
                )
            self.robot_list = []
            for i in range(self.robot_num):
                self.robot_list.append(
                    CBBA.agent(
                        id=i,
                        tasks=self.tasks,
                        agent_num=self.robot_num,
                        L_t=task_capacity,
                        state=initial_state,
                        point_estimation=use_point_estimation,
                    )
                )
        else:
            self.robot_list = agents

        self.communication_graph = coverage_problem.getCommunicationGraph()
        self.max_t = max_iterations
        self.plot = enable_plotting

    def evaluateSolution(self):
        travel_length = 0
        total_path_length = 0
        total_task_length = 0
        total_task_cost = 0
        route_list = []
        for r in self.robot_list:
            travel_length, task_length = r.getTotalPathCost()
            total_path_length += travel_length
            total_task_length += task_length
            total_task_cost += r.getTotalTravelCost(r.getPathTasks())
            route = [r.state.squeeze()]
            for task in r.getPathTasks():
                route.append(task.start)
                route.append(task.end)
            route.append(r.state.squeeze())
            route_list.append(route)

        print("Execution time: ", self.end_time - self.start_time)
        print("Total Path Length:", total_path_length)
        print("Total path cost:", total_task_cost)
        print("Total task Length:", total_task_length)
        print("Iterations: ", self.iterations)
        return (
            total_path_length,
            total_task_length,
            total_task_cost,
            self.iterations,
            self.end_time - self.start_time,
            route_list,
        )

    def solve(self, profiling_enabled=False, debug=False):
        if profiling_enabled:
            print("Profiling enabled!")
            import cProfile, pstats, io
            from pstats import SortKey

            pr = cProfile.Profile()
            pr.enable()
        t = 0  # Iteration number
        plotter = Utility.Plotter(self.tasks, self.robot_list, self.communication_graph)

        # Plot the search area and restricted area
        plotter.plotAreas([self.coverage_problem.getSearchArea()], color=(0, 0, 0, 0.5))
        plotter.plotAreas(
            self.coverage_problem.getRestrictedAreas(), color=(1, 0, 0, 0.2), fill=True
        )
        self.start_time = timeit.default_timer()

        while True:
            converged_list = []

            print("Iteration {}".format(t + 1))
            # Phase 1: Auction Process
            for robot in self.robot_list:
                robot.build_bundle()
            if debug:
                print("Bundle")
                for robot in self.robot_list:
                    print(robot.getBundle())
                print("Path")
                for robot in self.robot_list:
                    print(robot.getPath())

            # Plot
            # if self.plot:
            #     plotter.setTitle("Time Step:{}, Bundle Construct".format(t))
            #     for robot in self.robot_list:
            #         plotter.plotAgents(robot, self.tasks, t)
            #     plotter.pause(0.1)

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
            # if self.plot:
            #     plotter.setTitle("Time Step:{}, Consensus".format(t))
            #     for robot in self.robot_list:
            #         plotter.plotAgents(robot, self.tasks, t)

            #     plotter.pause(0.1)
            if debug:
                print("Bundle")
                for robot in self.robot_list:
                    print(robot.getBundle())
                print("Path")
                for robot in self.robot_list:
                    print(robot.getPath())

            t += 1

            if sum(converged_list) == self.robot_num or t >= self.max_t:
                break
        self.iterations = t

        if profiling_enabled:
            print("Profiling finished:")
            s = io.StringIO()
            sortby = SortKey.CUMULATIVE
            ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            ps.print_stats(100)
            pr.disable()

        print("Robot Routes")
        for robot in self.robot_list:
            print(robot.getPath())

        if self.plot:
            for robot in self.robot_list:
                plotter.plotAgents(robot, self.tasks, 0)

        self.end_time = timeit.default_timer()
        if self.plot:
            plotter.show()
