import timeit

import numpy as np
import shapely

from task_allocation import CBBA, Agent, CoverageProblem, Utility


class Runner:
    def __init__(self, coverage_problem: CoverageProblem.CoverageProblem, agents: list[Agent.agent], enable_plotting=False):
        # Task definition
        self.coverage_problem = coverage_problem
        self.robot_list = []
        # TODO add all agent positions to the travelgraph before creating the cbba agents!!!!!
        for agent in agents:
            self.robot_list.append(
                CBBA.agent(
                    id=agent.id,
                    state=shapely.Point(agent.position),
                    environment=self.coverage_problem.environment,
                    tasks=np.array(self.coverage_problem.getTasks()),
                    number_of_agents=len(agents),
                    capacity=agent.capacity,
                )
            )

        self.communication_graph = np.ones((len(agents), len(agents)))
        self.plot = enable_plotting
        self.progressed_time = 0

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
                route.append(task.start)
                route.append(task.end)
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
        """_summary_

        Parameters
        ----------
        profiling_enabled : bool, optional
            _description_, by default False
        debug : bool, optional
            _description_, by default False
        """
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
                # TODO parellalize this!
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

                Y = {neighbor_id: message_pool[neighbor_id] for neighbor_id in connected} if len(connected) > 0 else None

                robot.receive_message(Y)

            # Phase 2: Consensus Process
            for robot in self.robot_list:
                # Update local information and decision
                if Y is not None:
                    converged = robot.update_task()
                    converged_list.append(converged)

            if debug:
                # Plot
                if self.plot:
                    plotter.setTitle("Time Step:{}".format(t))
                    plotter.plotAgents(self.robot_list)
                    plotter.pause(0.1)

                print("Bundle")
                for robot in self.robot_list:
                    print(robot.getBundle())
                print("Path")
                for robot in self.robot_list:
                    print(robot.getPath())

            t += 1

            if sum(converged_list) == len(self.robot_list):
                break
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
        result = {}
        for robot in self.robot_list:
            print(robot.getPath())
            # TODO this should not be calculated here, but rather while solving...
            result[robot.id] = robot.getTravelPath()

        if self.plot:
            plotter.plotAgents(self.robot_list)
        if self.plot:
            plotter.show()

        return result

    def plan(self):
        while True:
            converged_list = []
            # Phase 1: Auction Process
            for robot in self.robot_list:
                robot.build_bundle()

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

                robot.receive_message(Y)

            # Phase 2: Consensus Process
            for robot in self.robot_list:
                # Update local information and decision
                if Y is not None:
                    converged = robot.update_task()
                    converged_list.append(converged)

            if sum(converged_list) == len(self.robot_list):
                break

    def replan(self):
        for r in self.robot_list:
            # For each task in the robots list which has been completed

            for p in r.path:
                # TODO figure out which tasks have already been completed by the agents and set reward to 0
                pass

        # Replan the problem using the new information
        self.plan()

    def add_agent(self, agent: Agent):
        # TODO: Add an agent to self.robot_list

        # Initialize a new agent using the existing tasks/agents

        # Update the existing agents matrices to include the new agent
        pass

    def add_time(self, time_in_seconds):
        self.progressed_time = time_in_seconds


class Evaluator:
    def __init__(self):
        pass
