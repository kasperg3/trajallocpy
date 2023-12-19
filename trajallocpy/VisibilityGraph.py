import math
from itertools import combinations, product

import networkx as nx
import numpy as np
from scipy.spatial import KDTree
from shapely import prepare
from shapely.geometry import LineString, MultiPolygon, Polygon

from trajallocpy import Utility


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


def cross_product(u, v):
    return u[0] * v[1] - u[1] * v[0]


@Utility.timing()
def visibility_graph(polygon: Polygon, holes: MultiPolygon, reduced_visibility=True):
    # Create a NetworkX graph to represent the visibility graph
    visibility_graph = construct_graph(polygon, holes)
    # Only compare the combinations which are unique
    node_combinations = list(combinations(visibility_graph.nodes, 2))
    # Performance improvements
    prepare(polygon)
    prepare(holes)

    if reduced_visibility:
        # reduced visibility graph
        edges = []
        for u, v in node_combinations:
            u_og = np.array(u)
            v_og = np.array(v)
            v_0 = v_og - u_og

            edges_0 = list(visibility_graph.edges(v))
            e_1 = np.array(edges_0[0][0]) - np.array(edges_0[0][1])
            e_2 = np.array(edges_0[1][0]) - np.array(edges_0[1][1])
            if cross_product(e_2, v_0) * cross_product(e_1, v_0) >= 0:
                edges.append((u, v))
            else:
                v_1 = u_og - v_og
                edges_0 = list(visibility_graph.edges(u))
                e_1 = np.array(edges_0[0][1]) - np.array(edges_0[0][0])
                e_2 = np.array(edges_0[1][1]) - np.array(edges_0[1][0])
                if cross_product(e_2, v_1) * cross_product(e_1, v_1) >= 0:
                    edges.append((u, v))
        node_combinations = edges

    for u, v in node_combinations:
        if is_visible(polygon, holes, u, v):
            visibility_graph.add_edge(u, v)

    # Add cost
    add_euclidean_cost_to_edges(graph=visibility_graph)

    return visibility_graph


def is_visible(polygon, holes, u, v):
    intersects_obstacle = False

    line = LineString([u, v])
    # Check if the line is outside the polygon and the line is not touching the polygon
    # "Touching" is not overlapping but has a common coordinate
    if not line.within(polygon) and not line.touches(polygon):
        intersects_obstacle = True
    elif not holes.is_empty:
        # Check if the line is intersecting any of the obstacles
        for obstacle in holes.geoms:
            # TODO line.crosses fails if the line is touching the obstacle
            if line.crosses(obstacle) or line.within(obstacle):
                intersects_obstacle = True
                break
    return not intersects_obstacle


def point_line_distance(point, line):
    """Calculates the Euclidean distance between a point and a line segment.

    Args:
        point: A tuple or list of two floats, representing the coordinates of the point.
        line: A tuple or list of two tuples or lists of two floats, representing the coordinates of the endpoints of the line segment.

    Returns:
        A float representing the Euclidean distance between the point and the line segment.
    """
    x1, y1 = line[0]
    x2, y2 = line[1]
    x0, y0 = point
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
    if denominator == 0:
        return 10000000000
    return numerator / denominator


def project_point_onto_line(point, endpoint1, endpoint2):
    """Calculates the projection of a point onto a line segment defined by two endpoints.

    Args:
        point: A tuple or list of two floats, representing the coordinates of the point.
        endpoint1: A tuple or list of two floats, representing the coordinates of the first endpoint.
        endpoint2: A tuple or list of two floats, representing the coordinates of the second endpoint.

    Returns:
        A tuple of two floats representing the coordinates of the projection point.
    """
    x1, y1 = endpoint1
    x2, y2 = endpoint2
    x0, y0 = point
    dx, dy = x2 - x1, y2 - y1
    dot = dx * (x0 - x1) + dy * (y0 - y1)
    length_squared = dx**2 + dy**2
    if length_squared == 0:
        return endpoint1
    t = max(0, min(1, dot / length_squared))
    x = x1 + t * dx
    y = y1 + t * dy
    return (x, y)


def point_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def add_euclidean_cost_to_edges(graph):
    # Add a cost based on the euclidean distance for each edge
    edge_attributes = {e: math.sqrt((e[1][0] - e[0][0]) ** 2 + (e[1][1] - e[0][1]) ** 2) for e in graph.edges()}
    nx.set_edge_attributes(
        G=graph,
        values=edge_attributes,
        name="cost",
    )


def add_points_to_graph(graph: nx.Graph, points, connect_to_visible_points=False, polygon=None, holes=None):
    nodes = list(graph.nodes())
    kdtree = KDTree(nodes)
    for point in points:
        add_point_to_graph(kdtree, graph, point)

    # edges to check for visibility
    if connect_to_visible_points:
        edges = list(product(points, graph.nodes()))
        # edges = list(combinations(graph.nodes(), 2))
        for u, v in edges:
            if is_visible(polygon, holes, u, v):
                graph.add_edge(u, v)

    # Add a cost based on the euclidean distance for each edge
    add_euclidean_cost_to_edges(graph=graph)


def add_edge(graph, point1, point2):
    if point1 != point2:
        graph.add_edge(point1, point2)


def add_point_to_graph(kdtree, graph: nx.Graph, new_point):
    # If the node is already in the graph, do not add it
    if new_point in graph.nodes():
        return

    # Create a KDTree from the nodes of the graph
    # nodes = list(graph.nodes())
    # # Find the nearest edge to the new point
    # _, nearest_node_index = kdtree.query(new_point)
    # nearest_node = nodes[nearest_node_index]
    # TODO investigate what happens if the new_point is at the endpoint of an edge
    nearest_edge = min(graph.edges(), key=lambda e: point_line_distance(new_point, e))

    # Calculate the projection of the new point onto the nearest edge
    endpoint1, endpoint2 = nearest_edge
    projection_point = project_point_onto_line(new_point, endpoint1, endpoint2)

    # Add the new point
    graph.add_node(new_point, pos=new_point)

    # TODO Figure out whether redundant/self refs. edges should be removed
    # If the projected point is the same point as a existing node, do not add an extra edge
    # if projection_point == nearest_node:
    #     add_edge(graph, new_point, nearest_node)
    # if projection_point == new_point:
    #     # point_distance(projection_point, new_point) < 0.1:
    #     # If the distance from the projection point to the new_point is 0
    #     # graph.remove_edge(endpoint1, endpoint2)
    #     add_edge(graph, new_point, endpoint1)
    #     add_edge(graph, new_point, endpoint2)
    # else:
    # Add the new node and connect it to the projection point and the new point
    # Remove the nearest edge from the graph and add two new edges to the projection point
    # graph.remove_edge(endpoint1, endpoint2)
    graph.add_node(projection_point, pos=projection_point)
    # TODO sometimes the projectionpoint and endpoint is the same, why is this the case?
    add_edge(graph, new_point, projection_point)
    add_edge(graph, projection_point, endpoint2)
    add_edge(graph, projection_point, endpoint1)
