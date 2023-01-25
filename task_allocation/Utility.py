import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from task_allocation import CoverageProblem
import json
from task_allocation import Task


class Plotter:
    def __init__(self, tasks, robot_list, communication_graph):
        self.fig, self.ax = plt.subplots()

        # Plot tasks
        self.plotTasks(tasks)

        # Plot agents
        robot_pos = np.array([r.state.tolist() for r in robot_list])
        self.ax.plot(robot_pos[0][0], robot_pos[0][1], "b*", label="Robot")

        # Plot communication paths
        for i in range(len(robot_list) - 1):
            for j in range(i + 1, len(robot_list)):
                if communication_graph[i][j] == 1:
                    self.ax.plot(
                        [robot_pos[i][0], robot_pos[j][0]],
                        [robot_pos[i][1], robot_pos[j][1]],
                        "g--",
                        linewidth=1,
                    )

        handles, labels = self.ax.get_legend_handles_labels()
        communication_label = Line2D([0], [0], color="g", linestyle="--", label="communication")
        handles.append(communication_label)
        self.ax.legend(handles=handles)
        self.assign_plots = []

    def plotAgents(self, robot, task, iteration):
        if len(robot.path) > 0:
            task_x = []
            task_y = []
            task_x.append(robot.state[0])
            task_y.append(robot.state[1])
            for s in task[robot.path]:
                task_x.append(s.start[0])
                task_x.append(s.end[0])
                task_y.append(s.start[1])
                task_y.append(s.end[1])

            self.x_data = task_x
            self.y_data = task_y
        else:
            self.x_data = [robot.state[0]]
            self.y_data = [robot.state[1]]

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
                y.append([p["longitude"], p["latitude"]])
            p = Polygon(y, facecolor=color)
            self.ax.add_patch(p)

    def plotTasks(self, tasks):
        # TODO Should be able to plot 2 and 1 dimentional tasks
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


def loadCoverageProblem(path) -> CoverageProblem.CoverageProblem:

    # Opening JSON file
    f = open(path)

    # returns JSON object as
    # a dictionary
    data = json.load(f)
    f.close()

    # TODO convert all the coordinates to UTM for accurate calculations!!!!
    search = data["search_area"]
    restricted = data["restricted_areas"]
    sweep = data["sweeps"]
    test = [[s["longitude"], s["latitude"]] for s in sweep]

    sweeps = list(zip(test[::2], test[1::2]))
    tasks = []
    i = 0
    for s in sweeps:
        tasks.append(Task.Task(start=np.array(s[0]), end=np.array(s[1]), task_id=i))
        i = i + 1
    return CoverageProblem.CoverageProblem(
        search_area=search, restricted_area=restricted, tasks=tasks
    )
