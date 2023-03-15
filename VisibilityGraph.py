import json

import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import LineString, Polygon

from task_allocation import Utility

# # Shapely crosses maybe?

# # Create a polygon
# polygon = Polygon([(0, 0), (1, 1), (1, 0)])

# # Create a line
# line = LineString([(0, 1), (1, 0)])


# # Iterate over the edges of the polygon
# for i in range(len(polygon.exterior.coords) - 1):
#     # Create a line for the current edge
#     edge = LineString([polygon.exterior.coords[i], polygon.exterior.coords[i + 1]])

#     # Check if the edge intersects the line
#     if edge.intersects(line):
#         # Print the edge
#         print(edge)


def is_visible():

    # if pw_i intersects the interior of the obstacle of which w_i is a vertex then return false
    pass


def visibility_graph(polygon, holes):
    print(polygon)
    print(holes)
    polygon = Polygon(shell=polygon, holes=holes)

    # Plot the polygon
    x, y = polygon.exterior.xy
    plt.plot(x, y)

    for hole in polygon.interiors:
        x, y = hole.xy
        plt.plot(x, y)

    plt.show()

    # TODO
    # 1. Create a line for each point and sort the line based on the clockwise angle
    # 2. Find the edges that are intersected by the line P,
    #    and store them in a balanced search tree, T, in the order in which they are intersected by P.

    # 3. for all vertices add w_i to the list of IS_VISIBLE() vertices if it is visible
    #    Insert into T the obstacle edges incident to w_i, that lie on the clockwise side of the hald.kube from p to wi
    #    Delete from T the edges incident to w_i on the counterclockwise side of the half-line from p to w_i


if __name__ == "__main__":
    # main()

    dataset_name = "AC300"

    files = Utility.getAllCoverageFiles(dataset_name)

    for file_name in files:
        with open(file_name) as json_file:
            data = json.load(json_file)
        visibility_graph(data["polygon"], data["holes"])
        break
