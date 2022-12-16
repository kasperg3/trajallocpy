import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from task_allocation import CoverageProblem
import json
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import geopandas as gpd


class Plotter:
    def __init__(self, task, robot_list, communication_graph):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim((-0.1, 1.1))
        self.ax.set_ylim((-0.1, 1.1))

        # Plot tasks
        self.ax.plot(task[:, 0], task[:, 1], "rx", label="Task")

        # Plot agents
        robot_pos = np.array([r.state.tolist() for r in robot_list])
        self.ax.plot(robot_pos[0], robot_pos[1], "b*", label="Robot")

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
        communication_label = Line2D(
            [0], [0], color="g", linestyle="--", label="communication"
        )
        handles.append(communication_label)
        self.ax.legend(handles=handles)
        self.assign_plots = []

    def plotAgents(self, robot, task, iteration):
        if len(robot.path) > 0:
            self.x_data = [robot.state[0]] + task[robot.path, 0].tolist()
            self.y_data = [robot.state[1]] + task[robot.path, 1].tolist()
        else:
            self.x_data = [robot.state[0]]
            self.y_data = [robot.state[1]]

        if iteration == 0:
            (assign_line,) = self.ax.plot(self.x_data, self.y_data, "k-", linewidth=1)
            self.assign_plots.append(assign_line)
        else:
            self.assign_plots[robot.id].set_data(self.x_data, self.y_data)

    def setTitle(self, title):
        self.ax.set_title(title)

    def show(self):
        plt.show()

    def pause(self, wait_time=0.1):
        plt.pause(wait_time)

    def plotAreas(self, areas, color, is_filled=False):
        # plot geodetic data
        polygon1 = Polygon(
            [
                (0, 5),
                (1, 1),
                (3, 0),
            ]
        )

        p = gpd.GeoSeries(polygon1)
        p.plot()
        plt.show()

    def plotTasks(self):
        # TODO Should be able to plot 2 and 1 dimentional tasks
        pass


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

    return CoverageProblem.CoverageProblem(
        search_area=search, restricted_area=restricted, sweeps=sweep
    )
