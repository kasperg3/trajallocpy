import json
import math

import matplotlib.pyplot as plt
import networkx as nx
from shapely.geometry import LineString, Polygon

from BST import BalancedBinarySearchTree, Node
from task_allocation import Utility


def on_segment(segment, w_i):
    x, y = w_i
    (x1, y1), (x2, y2) = segment
    # check if the point is on the line defined by the segment
    if (x - x1) * (y2 - y1) == (y - y1) * (x2 - x1):
        # check if the point is between the endpoints of the segment
        if (x1 < x < x2 or x2 < x < x1) and (y1 < y < y2 or y2 < y < y1):
            return True
    return False


def intersection(line1, line2, include_endpoints):
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0:
        return None  # lines are parallel
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    if not (0 <= ua <= 1 and 0 <= ub <= 1):  # Not including the endpoints
        return None  # intersection point is outside of line segments
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)
    return (x, y)


def does_intersect(seg1, seg2, include_endpoints):
    x1, y1 = seg1[0]
    x2, y2 = seg1[1]
    x3, y3 = seg2[0]
    x4, y4 = seg2[1]

    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3

    det = dx1 * dy2 - dx2 * dy1

    if det == 0:
        return False

    t1 = (dx2 * (y1 - y3) - dy2 * (x1 - x3)) / det
    t2 = (dx1 * (y1 - y3) - dy1 * (x1 - x3)) / det
    if include_endpoints:
        if 0 <= t1 <= 1 and 0 <= t2 <= 1:
            return True
    else:
        if 0 < t1 < 1 and 0 < t2 < 1:
            return True
    return False


def find_intersection_in_tree(edge, S: BalancedBinarySearchTree):
    # TODO traverse the tree in order and find if any edges are intersecting!
    return False


def line_intersects_polygon(line, polygon):
    """
    Checks if a line intersects the interior of a polygon.

    Parameters:
    line (tuple): A tuple of two points representing the line.
    polygon (list): A list of tuples representing the vertices
                    of the polygon in clockwise or counterclockwise order.

    Returns:
    bool: True if the line intersects with the interior of the polygon,
          False otherwise.
    """
    # Unpack the line coordinates.
    x1, y1 = line[0]
    x2, y2 = line[1]

    # Iterate over the edges of the polygon.
    for i in range(len(polygon)):
        # Get the coordinates of the current and next vertex of the polygon.
        x3, y3 = polygon[i]
        x4, y4 = polygon[(i + 1) % len(polygon)]

        # Calculate the denominator and numerator of the equations for the line
        # segments and check for collinearity.
        denom = ((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1))
        num1 = ((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3))
        num2 = ((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3))

        # Check if the line segments are collinear.
        if denom == 0:
            if (num1 == 0) and (num2 == 0):
                # The line segments are coincident.
                return True
            else:
                # The line segments are parallel.
                continue

        # Calculate the point of intersection of the line segments.
        t1 = num1 / denom
        t2 = num2 / denom
        x_intersect = x1 + t1 * (x2 - x1)
        y_intersect = y1 + t1 * (y2 - y1)

        # Check if the point of intersection is within the bounds of the line segments
        # and if it is not one of the endpoints of either line segment.
        if (
            (0 < t1 < 1)
            and (0 < t2 < 1)
            and ((x_intersect != x1) or (y_intersect != y1))
            and ((x_intersect != x2) or (y_intersect != y2))
            and ((x_intersect != x3) or (y_intersect != y3))
            and ((x_intersect != x4) or (y_intersect != y4))
        ):
            # The line segments intersect.
            return True

    # The line segment does not intersect with the polygon.
    return False


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
    p_wi = (p, (p[0] + 10000000, 0))  # TODO make sure that this is enough
    for edge in S.edges():
        intersection_point = intersection(p_wi, edge, True)
        if intersection_point is not None:
            T.insert(0.0, math.dist(p, intersection_point), edge)

    # Sort the list of points based on their angle to the origin
    w_previous = None
    w_previous_visible = None
    for w_i in sorted(S.nodes, key=angle_to_point):
        if w_i == p:  # Do not check collision between the same nodes
            continue

        p_wi = (p, w_i)
        visible = True

        # If p_wi intersects any edges of the polygon which w_i is a part of do not check anything else
        polygon_nodes = list(nx.dfs_preorder_nodes(S, w_i))

        visible = not line_intersects_polygon(p_wi, polygon_nodes)

        # polygon_edges = []
        # for i in range(len(polygon_nodes)):
        #     edge = (
        #         (polygon_nodes[i], polygon_nodes[0])
        #         if i == len(polygon_nodes) - 1
        #         else (polygon_nodes[i], polygon_nodes[i + 1])
        #     )
        #     polygon_edges.append(edge)

        # # Check for intersections with the nodes own polygon
        # for edge in polygon_edges:
        #     # TODO Check if this is correct!
        #     # do not check the incident edges to w_i
        #     if not (w_i == edge[0] or w_i == edge[1]):
        #         continue

        #     if does_intersect(edge, p_wi, include_endpoints=True):
        #         visible = False
        #         break

        if visible:
            # If it is the first iteration or w_i-1 is not on the segment p_wi
            if w_previous is None or not on_segment(p_wi, w_previous):
                # search in T for the edge e in the leaftmost leaf
                e = T.find_min()
                if e is not None:
                    visible = False  # not does_intersect(p_wi, e.edge, include_endpoints=False)
                # visible = not (e is not None and does_intersect(p_wi, e.edge, include_endpoints=True))
            elif w_previous_visible:
                # TODO search T for an edge that intersects the segment (w_i-1, w_i)
                visible = False  # find_intersection_in_tree((w_previous, w_i), S)

        if visible:
            W.append((p, w_i))
            for edge in S.edges(w_i):  # the point have two edges
                for node in edge:
                    # only compare with the node which is not the common point
                    if node is not w_i:
                        angle, distance = angle_to_point(node)
                        if angle_to_point(w_i) <= angle_to_point(node):
                            # Edge is on the Clockwise side of w_i
                            T.insert(angle, distance, edge)
                        else:
                            # Edge is on the Counter-clockwise side of w_i
                            T.delete(angle, distance)

        w_previous = w_i
        w_previous_visible = visible
    T.pretty_print()

    return W


def construct_graph(polygon: Polygon, holes: list):
    G = nx.Graph()
    for i in range(len(polygon.boundary.coords) - 1):
        node_from = polygon.boundary.coords[i]
        node_to = polygon.boundary.coords[i + 1]
        G.add_node(node_from, pos=node_from)
        G.add_node(node_to, pos=node_to)
        G.add_edge(polygon.boundary.coords[i], polygon.boundary.coords[i + 1], type="border")

    node_from = polygon.boundary.coords[-1]
    node_to = polygon.boundary.coords[len(polygon.boundary.coords) - 2]
    G.add_node(node_from, pos=node_from)
    G.add_node(node_to, pos=node_to)
    G.add_edge(polygon.boundary.coords[-1], polygon.boundary.coords[len(polygon.boundary.coords) - 2], type="border")

    for hole in holes:
        for i in range(len(hole.boundary.coords) - 1):
            node_from = hole.boundary.coords[i]
            node_to = hole.boundary.coords[i + 1]
            G.add_node(node_from, pos=node_from)
            G.add_node(node_to, pos=node_to)
            G.add_edge(hole.boundary.coords[i], hole.boundary.coords[i + 1], type="obstacle")

        node_from = hole.boundary.coords[-1]
        node_to = hole.boundary.coords[len(hole.boundary.coords) - 2]
        G.add_node(node_from, pos=node_from)
        G.add_node(node_to, pos=node_to)
        G.add_edge(hole.boundary.coords[-1], hole.boundary.coords[len(hole.boundary.coords) - 2], type="obstacle")
    return G


def visibility_graph(polygon: Polygon, holes: list):
    # Construct the graph
    G = construct_graph(polygon, holes)
    # Add all vertices and edges of the polygons to the graph
    print(polygon)
    print("The polygon ", G)
    # check for visibility in each of the vertices
    edges = []
    for v in G.nodes:
        edges.extend(visible_vertices(v, G))

    G.add_edges_from(edges)
    nx.draw(G, nx.get_node_attributes(G, "pos"), with_labels=True)
    plt.show()


if __name__ == "__main__":
    visibility_graph(
        Polygon([(0, 0), (0, 10), (10, 10), (10, 0)]),
        [
            Polygon([(2, 2), (4, 2), (4, 4), (2, 4)]),
            Polygon([(6, 2), (7, 2), (7, 6), (6, 6)]),
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
