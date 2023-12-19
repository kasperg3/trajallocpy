import os
import time

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as PolygonPatch

from trajallocpy import CBBA, Agent


def timing(f):
    def wrap(*args, **kwargs):
        start = time.time()
        result = f(*args, **kwargs)
        end = time.time()
        print(f"Elapsed time: {end - start} seconds")
        return result

    return wrap


class Plotter:
    def __init__(self, robot_list, communication_graph):
        self.fig, self.environmentAx = plt.subplots()
        # Plot agents
        robot_pos = np.array([r.state for r in robot_list])

        # Plot agent information
        for i in range(len(robot_list)):
            # # Plot communication graph path
            # for j in range(i + 1, len(robot_list)):
            #     if communication_graph[i][j] == 1:
            #         self.environmentAx.plot(
            #             [robot_pos[i][0], robot_pos[j][0]],
            #             [robot_pos[i][1], robot_pos[j][1]],
            #             "g--",
            #             linewidth=1,
            #         )
            # Plot agent position
            self.environmentAx.scatter(robot_pos[:, 0], robot_pos[:, 1], color="black")

        handles, labels = self.environmentAx.get_legend_handles_labels()
        communication_label = Line2D([0], [0], color="g", linestyle="--", label="communication")
        handles.append(communication_label)
        self.environmentAx.legend(handles=handles)
        self.assign_plots = []

    def plotAgents(self, robot_list: list[CBBA.agent]):
        for robot in robot_list:
            self.plotAgent(robot, len(robot_list))

    def plotAgent(self, robot: CBBA.agent, total_number_of_robots):
        task_x = []
        task_y = []
        p = Agent.getTravelPath(robot.state, robot.getPathTasks(), robot.environment)

        for s in p:
            task_x.append(s[0])
            task_y.append(s[1])

        self.x_data = task_x
        self.y_data = task_y

        if len(self.assign_plots) < total_number_of_robots:
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

    def save(self, filename):
        plt.savefig(filename)

    # def save_gif(self, filename):
    #     # Define the update function
    #     def update(frame):
    #         plt.cla()
    #         plt.plot(self.plot_history[frame])

    #     # Create the animation
    #     anim = FuncAnimation(plt.gcf(), update, frames=len(self.plot_history), repeat=True)

    #     # Save the animation as a GIF
    #     anim.save("animation.gif", writer="imagemagick")

    # import glob

    # from PIL import Image

    # def make_gif(frame_folder):
    #     frames = [Image.open(image) for image in sorted(glob.glob(f"*.png"), key=lambda x: int("".join(filter(str.isdigit, x))))]
    #     frames.append(frames[-1])
    #     frames.append(frames[-1])
    #     frames.append(frames[-1])
    #     frames.append(frames[-1])
    #     frame_one = frames[0]
    #     frame_one.save("my_awesome.gif", format="GIF", append_images=frames, save_all=True, duration=500, loop=0)

    # if __name__ == "__main__":
    #     make_gif("/path/to/images")

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
        elif os.path.isfile(filepath) and filepath.endswith(route_data_name) or filepath.endswith(outer_poly_name) or filepath.endswith(holes_name):
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


def plotGraph(G, boundary, obstacles, tasks=None):
    plt.figure("travel graph")
    options = {"edgecolors": "tab:gray", "node_size": 20, "alpha": 0.7}
    nx.draw_networkx_edges(G, nx.get_node_attributes(G, "pos"), width=1.0, alpha=0.5)
    nx.draw_networkx_nodes(G, nx.get_node_attributes(G, "pos"), node_color="tab:blue", **options)

    x, y = boundary.exterior.xy
    plt.plot(x, y)
    for poly in obstacles.geoms:
        xi, yi = poly.exterior.xy
        plt.plot(xi, yi)
    if tasks is not None:
        for t in tasks:
            plt.plot(
                *t.trajectory.xy,
                "b--",
                linewidth=1,
            )
    plt.pause(0.01)
    plt.pause(0.01)
