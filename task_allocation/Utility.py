import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from task_allocation import CoverageProblem
import json
from task_allocation import Task, CBBA
import logging as log
from functools import lru_cache, wraps


import csv
import os


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
                linewidth=1.5,
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

    def plotAreas(self, areas, color, fill=False):
        for a in areas:
            y = []
            for p in a:
                y.append([p[0], p[1]])
            p = Polygon(y, facecolor=color)
            p.set_fill(fill)
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


def loadRoutePlan(csvFilePath):
    with open(csvFilePath, "r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=" ")
        rows = []
        for row in csv_reader:
            rows.append(row)
        lines = []
        for i in range(len(rows)):
            row = rows[i]
            if row[2] == "1":
                next_row = rows[i + 1]
                lines.append(
                    [
                        [float(next_row[0]), float(next_row[1])],
                        [float(row[0]), float(row[1])],
                    ]
                )
        edgeDict = []
    for s in lines:
        edgeDict.append({"start": s[0], "end": s[1]})
    return {"lines": edgeDict}


def loadPolygonFile(polygon_file, holes_file):
    # Add the polygon
    with open(polygon_file, "r") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=" ")
        polygon = []
        for row in csv_reader:
            polygon.append([float(row[0]), float(row[1])])

    # add the holes
    hole_list = []
    if holes_file:
        with open(holes_file, "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=" ")
            hole = []
            for row in csv_reader:
                if not row:
                    hole_list.append(hole)
                    hole = []
                else:
                    hole.append([float(row[0]), float(row[1])])

            # Add the final hole
            hole_list.append(hole)

    polygon_with_holes_dict = {"polygon": polygon, "holes": hole_list}

    return polygon_with_holes_dict


def loadCoverageProblemFromDict(data, nr) -> CoverageProblem.CoverageProblem:
    polygon = data["polygon"]
    holes = data["holes"]
    lines = data["lines"]
    tasks = []
    # Convert tasks and shuffle the start and end points
    for i in range(len(lines)):
        s = lines[i]
        if np.random.choice(2, 1):
            tasks.append(Task.Task(start=np.array(s["start"]), end=np.array(s["end"]), task_id=i))
        else:
            tasks.append(Task.Task(start=np.array(s["end"]), end=np.array(s["start"]), task_id=i))

    return CoverageProblem.CoverageProblem(
        search_area=polygon, restricted_area=holes, tasks=tasks, number_of_robots=nr
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
