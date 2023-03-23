import json
import math
from itertools import combinations

import geojson
import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import (
    LineString,
    MultiLineString,
    MultiPolygon,
    Point,
    Polygon,
    shape,
)

from BST import BalancedBinarySearchTree, Node
from task_allocation import Utility


@Utility.timing()
def construct_graph(polygon: Polygon, holes: MultiPolygon):
    G = nx.Graph()
    for i in range(len(polygon.boundary.coords) - 1):
        node_from = polygon.boundary.coords[i]
        node_to = polygon.boundary.coords[i + 1]
        G.add_node(node_from, pos=node_from, type="border")
        G.add_node(node_to, pos=node_to, type="border")
        G.add_edge(polygon.boundary.coords[i], polygon.boundary.coords[i + 1], type="border")

    node_from = polygon.boundary.coords[-1]
    node_to = polygon.boundary.coords[len(polygon.boundary.coords) - 2]
    G.add_node(node_from, pos=node_from, type="border")
    G.add_node(node_to, pos=node_to, type="border")
    G.add_edge(polygon.boundary.coords[-1], polygon.boundary.coords[len(polygon.boundary.coords) - 2], type="border")
    if not holes.is_empty:
        for hole in holes.geoms:
            for i in range(len(hole.boundary.coords) - 1):
                node_from = hole.boundary.coords[i]
                node_to = hole.boundary.coords[i + 1]
                G.add_node(node_from, pos=node_from, type="obstacle")
                G.add_node(node_to, pos=node_to, type="obstacle")
                G.add_edge(hole.boundary.coords[i], hole.boundary.coords[i + 1], type="obstacle")

            node_from = hole.boundary.coords[-1]
            node_to = hole.boundary.coords[len(hole.boundary.coords) - 2]
            G.add_node(node_from, pos=node_from, type="obstacle")
            G.add_node(node_to, pos=node_to, type="obstacle")
            G.add_edge(hole.boundary.coords[-1], hole.boundary.coords[len(hole.boundary.coords) - 2], type="obstacle")
    return G


# Sweep a line through the space stopping at vertices which are called events


# Maintain a list L of the current edges the slice intersects

# Determining the intersection of slice with L requires O(n) time but with
# an efficient data structure like a balanced tree, perhaps O(log n)

# Really, determine between which two edges the vertex or event lies These edges are e_lower, e_upper

# So, really maintaining L takes O(n log n) – log n for insertions, n for vertices

# events:

# Out: The two cells are closed and one is opened
# e_lower and e_upper are both to the left of the sweep line
#   * Delete e_lower and e_upper from the list

# In: The current cell is closed and 2 new cells are opened
# e_lower and e_upper are both to the right of the sweep line
#   * insert e_lower and e_upper to the list

# Middel: TCD(the current cell is closed and a new is opened)
#   * e_lower is to the left and e_upper is to the right of the sweep line
#       *
#   * e_lower is to the right and e_upper is to the left of the sweep line
#       * delete e_upper from the list and insert e_lower


def process_event(G, node, sorted_nodes, processed_vertices, edge_list, open_cells, closed_cells):
    # Get e_lower and e_upper
    edges = list(G.edges(node))

    # Define the vertical line and the intersections
    e_prev = edges[0]
    e_next = edges[1]

    # catch any vertical edges
    if e_prev[0][0] == e_prev[1][0]:
        temp = list(G.edges(e_prev[0]))[1]
        e_prev = (temp[1], temp[0])
    elif e_next[0][0] == e_next[1][0]:
        # if the line is vertical choose the previous edge leading to the node
        e_next = list(G.edges(e_next[1]))[0]

    if e_prev[1][0] < e_prev[0][0] and e_next[1][0] < e_next[0][0]:  # Out
        print("OUT: ", e_prev, " ", e_next)
    elif e_prev[1][0] > e_prev[0][0] and e_next[1][0] > e_next[0][0]:  # IN
        print("IN: ", e_prev, " ", e_next)
    else:  # Middel
        print("Middel", e_prev, " ", e_next)


def trapezoidal_decomposition(polygon, holes):

    G = construct_graph(geometries["boundary"], geometries["obstacles"])

    # Sort the vertices based on the x values, tie breaking with the y value
    nodes = sorted(G.nodes(), key=lambda node: (node[0], node[1]))

    L = []
    open_cells = []
    closed_cells = []
    processed_vertices = []

    for node in nodes:
        if node in processed_vertices:
            continue
        process_event(G, node, nodes, processed_vertices, L, open_cells, closed_cells)
    options = {"edgecolors": "tab:gray", "node_size": 100, "alpha": 0.9}

    nx.draw_networkx_edges(G, nx.get_node_attributes(G, "pos"), width=1.0, alpha=0.5)
    nx.draw_networkx_nodes(G, nx.get_node_attributes(G, "pos"), node_color="tab:blue", **options)
    plt.show()


dataset_name = "AC300"

files = Utility.getAllCoverageFiles(dataset_name)

for file_name in files:
    with open(file_name) as json_file:
        features = geojson.load(json_file)["features"]

    # Convert each feature to a Shapely geometry object
    geometries = {}
    for feature in features:
        geometries[feature["id"]] = shape(feature["geometry"])

    # Create a GeometryCollection with the geometries and their types
    decomposition = trapezoidal_decomposition(geometries["boundary"], geometries["obstacles"])
    break
