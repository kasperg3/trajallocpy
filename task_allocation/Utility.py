import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from task_allocation import CoverageProblem
import json
from task_allocation import Task, CBBA
import logging as log

import utm


class Plotter:
    def __init__(self, tasks, robot_list, communication_graph):
        self.fig, self.ax = plt.subplots()

        # Plot tasks
        self.plotTasks(tasks)

        # Plot agents
        robot_pos = np.array([r.state.tolist() for r in robot_list])

        # Plot agent information
        for i in range(len(robot_list)):
            # Plot communication graph path
            for j in range(i + 1, len(robot_list)):
                if communication_graph[i][j] == 1:
                    self.ax.plot(
                        [robot_pos[i][0], robot_pos[j][0]],
                        [robot_pos[i][1], robot_pos[j][1]],
                        "g--",
                        linewidth=1,
                    )
            # Plot agent position
            self.ax.plot(robot_pos[i][0], robot_pos[i][1], "b*", label="Robot")

        handles, labels = self.ax.get_legend_handles_labels()
        communication_label = Line2D([0], [0], color="g", linestyle="--", label="communication")
        handles.append(communication_label)
        self.ax.legend(handles=handles)
        self.assign_plots = []

    def plotAgents(self, robot: CBBA.agent, task, iteration):
        task_x = [robot.state[0]]
        task_y = [robot.state[1]]
        for s in robot.getPathTasks():
            task_x.append(s.start[0])
            task_x.append(s.end[0])
            task_y.append(s.start[1])
            task_y.append(s.end[1])

        self.x_data = task_x
        self.y_data = task_y

        if iteration == 0:
            (assign_line,) = self.ax.plot(
                self.x_data,
                self.y_data,
                linestyle="solid",
                color=robot.color,
                linewidth=1,
            )
            self.assign_plots.append(assign_line)
        else:
            self.assign_plots[robot.id].set_data(self.x_data, self.y_data)

    def setTitle(self, title):
        self.ax.set_title(title)

    def show(self):
        plt.show()

    def pause(self, wait_time):
        plt.pause(wait_time)

    def plotAreas(self, areas, color, is_filled=False):
        for a in areas:
            y = []
            for p in a:
                easting, northing, _, _ = utm.from_latlon(
                    p["longitude"], p["latitude"], force_zone_number=40, force_zone_letter="U"
                )
                y.append([easting, northing])
            p = Polygon(y, facecolor=color)
            self.ax.add_patch(p)

    def plotTasks(self, tasks):
        for t in tasks:
            self.ax.plot(
                [
                    t.start[0],
                    t.end[0],
                ],
                [
                    t.start[1],
                    t.end[1],
                ],
                "b--",
                linewidth=1,
            )


def loadCoverageProblem(path, nr) -> CoverageProblem.CoverageProblem:

    # Opening JSON file
    f = open(path)

    # returns JSON object as
    # a dictionary
    data = json.load(f)
    f.close()

    # convert all the coordinates to utm coordinates
    search = data["search_area"]
    restricted = data["restricted_areas"]

    # Convert the sweeps
    sweep = data["sweeps"]
    test = [
        utm.from_latlon(s["longitude"], s["latitude"], force_zone_number=40, force_zone_letter="U")
        for s in sweep
    ]
    sweeps = list(zip(test[::2], test[1::2]))
    tasks = []
    i = 0
    for s in sweeps:
        if np.random.choice(2, 1):
            tasks.append(Task.Task(start=np.array(s[0][:2]), end=np.array(s[1][:2]), task_id=i))
        else:
            tasks.append(Task.Task(start=np.array(s[1][:2]), end=np.array(s[0][:2]), task_id=i))
        i = i + 1
    log.info("Loaded %i tasks from file %s", i, "path")

    return CoverageProblem.CoverageProblem(
        search_area=search, restricted_area=restricted, tasks=tasks, number_of_robots=nr
    )
