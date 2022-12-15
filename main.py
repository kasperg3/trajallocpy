from task_allocation import CBBA, utility
import numpy as np

import os

# import from csv
coverage_problem = utility.loadCoverageProblem("data/OdenseSO.json")

# TODO plot the search area

# TODO Plot the restricted areas

# TODO plot the sweep tasks

len(coverage_problem.getSweeps())  # Task num

# Task definition
task_num = 20
robot_num = 4
task = np.random.uniform(low=0, high=1, size=(task_num, 2))

task_capacity = 10  # task.shape[0]

robot_list = [
    CBBA.agent(id=i, task_num=task_num, agent_num=robot_num, L_t=task_capacity)
    for i in range(robot_num)
]

# Initialize communication network
communication_graph = np.ones((robot_num, robot_num))  # Fully connected network
communication_graph[2, 3] = 0
communication_graph[3, 2] = 0
communication_graph[1, 2] = 0
communication_graph[3, 1] = 0

max_t = 100
plot_gap = 0.0

save_gif = False
filenames = []
t = 0  # Iteration number

plotter = utility.Plotter(task, robot_list, communication_graph)

while True:
    converged_list = []

    print("Iteration {}".format(t))
    # Phase 1: Auction Process
    plotter.set_title("Time Step:{}, Bundle Construct".format(t))
    for robot in robot_list:
        # select task by local information
        robot.build_bundle(task)
        # Plot
        plotter.plot_agents(robot, task, t)

    plotter.pause()

    # Communication stage
    # Send winning bid list to neighbors (depend on env)
    message_pool = [robot.send_message() for robot in robot_list]

    for robot_id, robot in enumerate(robot_list):
        # Recieve winning bidlist from neighbors
        g = communication_graph[robot_id]

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
    plotter.set_title("Time Step:{}, Consensus".format(t))
    for robot in robot_list:
        # Update local information and decision
        if Y is not None:
            converged = robot.update_task()
            converged_list.append(converged)
        # Plot
        plotter.plot_agents(robot, task, t)

    plotter.pause()
    t += 1

    if sum(converged_list) == robot_num or t > max_t:
        break

plotter.show()
