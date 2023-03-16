import json
import math

import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import LineString, Polygon

from BST import BalancedBinarySearchTree, Node
from task_allocation import Utility


def intersects_obstacle(w_i):
    pass


def intersects(p_wi, e):
    pass


def on_segment(segment, w_i):
    pass


def visible(w_current, w_prev, p_wi, w_prev_is_visible):
    # if pwi intersects the polygon of which it is a part of, return false
    if intersects_obstacle(w_current):
        return False
    elif on_segment(p_wi, w_prev) or w_prev is None:
        e = None
        # TODO search the tree for the left most leaf (the vertex closest to the point)

        # return not (e is not None and intersects(p_wi, e))
        if e is not None and intersects(p_wi, e):
            return False
        else:
            return True
    elif not w_prev_is_visible:
        e = None
        # Todo search for an edge e which intersects the line between w_prev and w_current

        return e is None


def intersect(line1, line2):
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0:
        return None  # lines are parallel
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
    if not (0 < ua < 1 and 0 < ub < 1):  # Not including the endpoints
        return None  # intersection point is outside of line segments
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)
    return (x, y)


def visible_vertices(p, S):
    """returns a set of visible vertices from point
    Parameters
    ----------
    holes : Set of obstacles
    point : a point
    Returns
    ----------
    [(point, visible_vertex, weight), ...]

    """
    # initialize the BST and result list
    W = []
    T = BalancedBinarySearchTree()
    # sort the obstacle vertices by the angle from initial rho to the vertex
    # in the case of a draw use the distance to the vertex as a tie breaker

    # Define a custom key function to calculate the angle of each point
    def angle_to_point(point):
        x, y = point
        angle = math.atan2(y - p[1], x - p[0]) * 180 / math.pi
        distance = math.dist(point, p)
        return ((360 - angle) % 360, distance)

    # Find the initial intersections and store then in a balanced search tree in the order that they were intersected by the line rho
    # Initialize the half-line rho to be pointing in the positive x direction
    rho = (p, (p[0] + 10000000, 0))  # TODO make sure that this is enough!
    # find the intersection with the initial line
    for edge in S.edges():
        intersection = intersect(rho, edge)
        if intersection is not None:
            # TODO add to BST in the order which the were intersected by rho
            dist = math.dist(p, intersection)
            T.insert(0.0, dist, edge)

    # Sort the list of points based on their angle to the origin
    for w_i in sorted(S.nodes, key=angle_to_point):
        if True:  # visible(w_i):
            for edge in S.edges(w_i):
                angle, distance = angle_to_point(edge[1])
                if angle_to_point(w_i) <= angle_to_point(edge[1]):
                    T.insert(angle, distance, edge)
                else:
                    T.delete(angle, distance)

    T.pretty_print()

    return W


def visibility_graph(polygon: Polygon, holes: list):
    # Construct the graph
    G = nx.Graph()
    # Add all vertices and edges of the polygons to the graph
    print(polygon)
    # TODO test whether it works for polygons with holes
    for i in range(len(polygon.boundary.coords) - 1):
        G.add_edge(polygon.boundary.coords[i], polygon.boundary.coords[i + 1], type="border")
    G.add_edge(polygon.boundary.coords[-1], polygon.boundary.coords[len(polygon.boundary.coords) - 2], type="border")

    for hole in holes:
        for i in range(len(hole.boundary.coords) - 1):
            G.add_edge(hole.boundary.coords[i], hole.boundary.coords[i + 1], type="obstacle")
        G.add_edge(hole.boundary.coords[-1], hole.boundary.coords[len(hole.boundary.coords) - 2], type="obstacle")

    print("The polygon ", G)
    # check for visibility in each of the vertices
    for v in G.nodes:
        G.add_edges_from(visible_vertices(v, G))

    nx.draw(G)
    plt.show()


if __name__ == "__main__":
    visibility_graph(
        Polygon([(0, 0), (0, 10), (10, 10), (10, 0)]),
        [
            Polygon([(0, 0.5), (1, 0), (1, 1), (0, 1)]),
            Polygon([(2, 0), (3, 0), (3, 1), (2, 1)]),
            Polygon([(4, 0), (5, 0), (5, 1), (4, 1)]),
        ],
    )
    # main()

    # dataset_name = "AC300"

    # files = Utility.getAllCoverageFiles(dataset_name)

    # for file_name in files:
    #     with open(file_name) as json_file:
    #         data = json.load(json_file)
    #     visibility_graph(data["polygon"], data["holes"])
    #     break
    #     break
    #     break
