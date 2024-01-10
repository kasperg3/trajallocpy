import timeit

import numpy as np
import shapely

from trajallocpy import ACBBA, CBBA, Agent, CoverageProblem, Utility


class Runner:
    def __init__(self, coverage_problem: CoverageProblem.CoverageProblem, agents: list[Agent.config], enable_plotting=False):
        # Task definition
        self.coverage_problem = coverage_problem
        self.robot_list = []

        for agent in agents:
            self.robot_list.append(
                ACBBA.agent(
                    id=agent.id,
                    state=shapely.Point(agent.position),
                    environment=self.coverage_problem.environment,
                    tasks=np.array(self.coverage_problem.getTasks()),
                    capacity=agent.capacity,
                    # number_of_agents=len(agents),
                    # point_estimation=False,
                )
            )

        self.communication_graph = np.ones((len(agents), len(agents)))
        self.plot = enable_plotting

    def evaluateSolution(self):
        total_path_length = 0
        total_task_length = 0
        total_path_cost = 0
        route_list = []
        max_path_cost = 0
        for r in self.robot_list:
            total_path_length += ACBBA.getTotalPathLength(r.state, r.getPathTasks(), r.environment)
            total_task_length += ACBBA.getTotalTaskLength(r.getPathTasks())
            agent_path_cost = ACBBA.getTotalTravelCost(r.state, r.getPathTasks(), r.environment)
            total_path_cost += agent_path_cost
            route = [r.state]
            for task in r.getPathTasks():
                route.extend(list(task.trajectory.coords))
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

    def add_tasks(self, tasks):
        # TODO make sure that the tasks are within the search area

        # TODO make sure that the tasks not already in the list
        for robot in self.robot_list:
            robot.add_tasks(tasks)

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

        if self.plot:
            plotter = Utility.Plotter(self.robot_list, self.communication_graph)
            # Plot the search area and restricted area
            plotter.plotPolygon(self.coverage_problem.getSearchArea(), color=(0, 0, 0, 0.5))
            plotter.plotMultiPolygon(self.coverage_problem.getRestrictedAreas(), color=(0, 0, 0, 0.2), fill=True)
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
                    print(robot.bundle)
                print("Path")
                for robot in self.robot_list:
                    print(robot.path)

            # Communication stage
            # Send winning bid list to neighbors (depend on env)
            message_pool = [robot.send_message() for robot in self.robot_list]
            for robot_id, robot in enumerate(self.robot_list):
                # Recieve winning bidlist from neighbors
                g = self.communication_graph[robot_id]

                (connected,) = np.where(g == 1)
                connected = list(connected)
                connected.remove(robot_id)

                Y = {neighbor_id: message_pool[neighbor_id] for neighbor_id in connected} if len(connected) > 0 else None
                robot.Y = Y

            # Phase 2: Consensus Process
            if isinstance(self.robot_list[0], ACBBA.agent):  # ACBBA
                messages = 0
                for robot in self.robot_list:
                    # Update local information and decision
                    messages += len(robot.update_task(robot.Y))

                if messages == 0:
                    break
            else:  # CBBA
                converged_list = []
                for robot in self.robot_list:
                    if Y is not None:
                        converged = robot.update_task()
                        converged_list.append(converged)

                if sum(converged_list) == len(self.robot_list):
                    break

            if debug:
                # Plot
                if self.plot:
                    plotter.setTitle("Time Step:{}".format(t))
                    plotter.plotAgents(self.robot_list)
                    plotter.pause(0.1)
                    # plotter.save("iteration{}.png".format(t))

                print("Bundle")
                for robot in self.robot_list:
                    print(robot.bundle)
                print("Path")
                for robot in self.robot_list:
                    print(robot.path)

            t += 1

        self.iterations = t

        if profiling_enabled:
            print("Profiling finished:")
            s = io.StringIO()
            sortby = SortKey.CUMULATIVE
            ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            ps.print_stats(100)
            pr.disable()

        self.end_time = timeit.default_timer()

        print("Robot Routes")
        routes = []
        for robot in self.robot_list:
            routes.append(Agent.getTrajectory(robot.getPathTasks()))

        print(routes)
        if self.plot:
            plotter.plotAgents(self.robot_list)
        if self.plot:
            plotter.show()

        return routes
