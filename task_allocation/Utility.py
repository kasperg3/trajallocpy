import os
import time

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as PolygonPatch

from task_allocation import CBBA


class Plotter:
    def __init__(self, tasks, robot_list, communication_graph):
        self.fig, self.environmentAx = plt.subplots()

        # Plot tasks
        self.plotTasks(tasks)

        # Plot agents
        robot_pos = np.array([r.state for r in robot_list])

        # Plot agent information
        for i in range(len(robot_list)):
            # Plot communication graph path
            for j in range(i + 1, len(robot_list)):
                if communication_graph[i][j] == 1:
                    self.environmentAx.plot(
                        [robot_pos[i][0], robot_pos[j][0]],
                        [robot_pos[i][1], robot_pos[j][1]],
                        "g--",
                        linewidth=1,
                    )
            # Plot agent position
            self.environmentAx.scatter(robot_pos[:, 0], robot_pos[:, 1], color="black")

        handles, labels = self.environmentAx.get_legend_handles_labels()
        communication_label = Line2D([0], [0], color="g", linestyle="--", label="communication")
        handles.append(communication_label)
        self.environmentAx.legend(handles=handles)
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
            (assign_line,) = self.environmentAx.plot(
                self.x_data,
                self.y_data,
                linestyle="solid",
                color=robot.color,
                linewidth=1.5,
            )
            self.assign_plots.append(assign_line)
        else:
            self.assign_plots[robot.id].set_data(self.x_data, self.y_data)

    def setTitle(self, title):
        self.environmentAx.set_title(title)

    def show(self):
        plt.show()

    def pause(self, wait_time):
        plt.pause(wait_time)

    def plotMultiPolygon(self, areas, color, fill=False):
        for a in areas.geoms:
            self.plotPolygon(a, color, fill)

    def plotPolygon(self, poly, color, fill=False):
        coords = []
        for x, y in poly.boundary.coords:
            coords.append([x, y])
        p = PolygonPatch(coords, facecolor=color)
        p.set_fill(fill)
        self.environmentAx.add_patch(p)

    def plotTasks(self, tasks):
        for t in tasks:
            self.environmentAx.plot(
                *t.trajectory.xy,
                "b--",
                linewidth=1,
            )


def loadDataset(directory, route_data_name, holes_name, outer_poly_name):
    csv_files = []
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        # Check if it is a directory
        if os.path.isdir(filepath):
            csv_files += loadDataset(filepath, route_data_name, holes_name, outer_poly_name)
        elif (
            os.path.isfile(filepath)
            and filepath.endswith(route_data_name)
            or filepath.endswith(outer_poly_name)
            or filepath.endswith(holes_name)
        ):
            csv_files.append(filepath)
    return csv_files


def getAllCoverageFiles(dataset, directory="CoverageTasks/"):
    result = []
    for filename in os.listdir(directory + dataset):
        result.append(os.path.join(directory + dataset, filename))
    return result


def timing(name=None):
    def decorator(func):
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            func_name = name if name is not None else func.__name__
            print(f"{func_name} execution time:{end_time - start_time} seconds")
            return result

        return wrapper

    return decorator


# TODO create a function to plot graphs
def plotGraph(G, boundary, obstacles):
    options = {"edgecolors": "tab:gray", "node_size": 50, "alpha": 0.7}
    nx.draw_networkx_edges(G, nx.get_node_attributes(G, "pos"), width=1.0, alpha=0.5)
    nx.draw_networkx_nodes(G, nx.get_node_attributes(G, "pos"), node_color="tab:blue", **options)

    x, y = boundary.exterior.xy
    plt.plot(x, y)
    for poly in obstacles.geoms:
        xi, yi = poly.exterior.xy
        plt.plot(xi, yi)
    plt.show()
    plt.clf()
