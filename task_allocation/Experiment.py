import timeit

import numpy as np

from task_allocation import CBBA, CoverageProblem, Utility


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
        self.task_num = int(len(self.tasks))  # number of geoms
        self.robot_num = self.coverage_problem.getNumberOfRobots()

        # TODO sampling of initial state should be based able to be based on distance/batterylife
        if agents is None:
            # if initial_state is None:
            initial_state = self.coverage_problem.generate_random_point_in_problem()
            self.robot_list = []
            # TODO construct task objects from the self.tasks list
            for i in range(self.robot_num):
                self.robot_list.append(
                    CBBA.agent(
                        id=i,
                        state=initial_state,
                        tasks=self.tasks,
                        agent_num=self.robot_num,
                        L_t=task_capacity,
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
        total_path_cost = 0
        route_list = []
        max_path_cost = 0
        for r in self.robot_list:
            travel_length, task_length = r.getTotalPathCost()
            total_path_length += travel_length
            total_task_length += task_length
            agent_path_cost = r.getTotalTravelCost(r.getPathTasks())
            total_path_cost += agent_path_cost
            route = [r.state]
            for task in r.getPathTasks():
                route.append(task.getStart())
                route.append(task.getEnd())
                # TODO add all points in the line, it is not necesarily single line tasks
            route.append(r.state)
            route_list.append(route)

            # Save the highest route cost
            if agent_path_cost > max_path_cost:
                max_path_cost = agent_path_cost

        print("Execution time: ", self.end_time - self.start_time)
        print("Total Path Length:", total_path_length)
        print("Total path cost:", total_path_cost)
        print("Total task Length:", total_task_length)
        print("Highest path cost:", max_path_cost)
        print("Iterations: ", self.iterations)
        return (
            total_path_length,
            total_task_length,
            total_path_cost,
            self.iterations,
            self.end_time - self.start_time,
            route_list,
            max_path_cost,
        )

    def solve(self, profiling_enabled=False, debug=False):
        if profiling_enabled:
            print("Profiling enabled!")
            import cProfile
            import io
            import pstats
            from pstats import SortKey

            pr = cProfile.Profile()
            pr.enable()
        t = 0  # Iteration number
        plotter = Utility.Plotter(self.tasks, self.robot_list, self.communication_graph)

        # Plot the search area and restricted area
        plotter.plotPolygon(self.coverage_problem.getSearchArea(), color=(0, 0, 0, 0.5))
        plotter.plotMultiPolygon(self.coverage_problem.getRestrictedAreas(), color=(1, 0, 0, 0.2), fill=True)
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

            # Communication stage
            # Send winning bid list to neighbors (depend on env)
            message_pool = [robot.send_message() for robot in self.robot_list]
            for robot_id, robot in enumerate(self.robot_list):
                # Recieve winning bidlist from neighbors
                g = self.communication_graph[robot_id]

                (connected,) = np.where(g == 1)
                connected = list(connected)
                connected.remove(robot_id)

                Y = (
                    {neighbor_id: message_pool[neighbor_id] for neighbor_id in connected}
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
                    plotter.plotAgents(robot, self.tasks, t)
                plotter.pause(0.1)

            if debug:
                print("Bundle")
                for robot in self.robot_list:
                    print(robot.getBundle())
                print("Path")
                for robot in self.robot_list:
                    print(robot.getPath())

            t += 1

            if sum(converged_list) == self.robot_num:
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
