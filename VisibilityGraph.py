import json
import math
from itertools import combinations

import geojson
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
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

    if include_endpoints:
        if not (0 <= ua <= 1 and 0 <= ub <= 1):  # Not including the endpoints
            return None  # intersection point is outside of line segments
    else:
        if not (0 < ua < 1 and 0 < ub < 1):  # Not including the endpoints
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


def inorder_traversal(current_node):
    if current_node:
        inorder_traversal(current_node.left)
        print(current_node)
        inorder_traversal(current_node.right)


def search_for_edge(root: Node, key):
    result = True
    if root is None:
        return False
    elif root is not None and does_intersect(root.edge, key, False):
        return True
    else:
        result = search_for_edge(root.left, key) or search_for_edge(root.right, key)
    return result


def find_intersection_in_tree(edge, tree: BalancedBinarySearchTree):
    return search_for_edge(tree.root, edge)


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
    p_wi = (p, (p[0] + 10000000, 0))
    for edge in S.edges():
        intersection_point = intersection(p_wi, edge, True)
        if intersection_point is not None:
            key = (0.0, math.dist(p, intersection_point))
            T.insert(key, edge)

    # Sort the list of points based on their angle to the origin
    w_previous = None
    w_previous_visible = None
    for w_i in sorted(S.nodes, key=angle_to_point):
        if w_i == p:  # Do not check collision between the same nodes
            continue

        p_wi = (p, w_i)
        visible = is_visible(S, T, p_wi, w_previous, w_previous_visible, w_i)
        if visible:
            W.append((p, w_i))
        # BST upkeep
        for edge in S.edges(w_i):  # the point have two edges
            for node in edge:
                # only compare with the node which is not the common point
                if node is not w_i:
                    angle, distance = angle_to_point(node)
                    if angle_to_point(w_i) <= angle_to_point(node):
                        # Edge is on the Clockwise side of w_i
                        T.insert((angle, distance), edge)
                    else:
                        # Edge is on the Counter-clockwise side of w_i
                        T.delete((angle, distance))
        w_previous = w_i
        w_previous_visible = visible

    # T.pretty_print()

    return W


def is_visible(S, T, p_wi, w_previous, w_previous_visible, w_i):
    visible = True
    # If p_wi intersects any edges of the polygon which w_i is a part of do not check anything else
    polygon_nodes = list(nx.dfs_preorder_nodes(S, w_i))
    poly = Polygon(polygon_nodes)
    line = LineString(p_wi)
    visible = not (line.crosses(poly)) and not line.within(poly)

    if visible:
        # If it is the first iteration or w_i-1 is not on the segment p_wi
        if w_previous is None or not on_segment(p_wi, w_previous):
            # search in T for the edge e in the leaftmost leaf
            # visible = not find_intersection_in_tree(p_wi, T)
            # TODO is this the right implementation?
            e = T.find_min()
            if e is not None:
                visible = not does_intersect(p_wi, e.edge, include_endpoints=True)
        elif w_previous_visible:
            # search T for an edge that intersects the segment (w_i-1, w_i)
            # no intersections == visible
            visible = not find_intersection_in_tree((w_previous, w_i), T)
        else:
            visible = False
    return visible


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


@Utility.timing()
def visibility_graph(polygon: Polygon, holes: list):
    # Construct the graph
    G = construct_graph(polygon, holes)
    # Add all vertices and edges of the polygons to the graph
    print(polygon)
    print(holes)
    print("Polygon as a graph: ", G)
    # check for visibility in each of the vertices
    edges = []
    for v in G.nodes:
        edges.extend(visible_vertices(v, G))

    G.add_edges_from(edges)

    # Add a cost based on the euclidean distance for each edge
    nx.set_edge_attributes(
        G,
        {e: ((e[0][0] - e[1][0]) ** 2 + (e[0][1] - e[0][1]) ** 2) ** 0.5 for e in G.edges()},
        "cost",
    )
    return G


def cross_product(u, v):
    return u[0] * v[1] - u[1] * v[0]


@Utility.timing()
def naive_visibility_graph(polygon: Polygon, holes: MultiPolygon, reduced_visibility=True):
    # Create a NetworkX graph to represent the visibility graph
    visibility_graph = construct_graph(polygon, holes)
    # Only compare the combinations which are unique
    # TODO to speed this up further check out the shapely prepare
    node_combinations = list(combinations(visibility_graph.nodes, 2))

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
        line = LineString([u, v])

        intersects_obstacle = False

        # Check if the line is outside the polygon
        if not line.within(polygon):
            intersects_obstacle = True
        elif not holes.is_empty:
            # Check if the line is intersecting any of the obstacles
            for obstacle in holes.geoms:
                if line.crosses(obstacle) or line.within(obstacle):
                    intersects_obstacle = True
                    break

        if not intersects_obstacle:
            visibility_graph.add_edge(u, v)

    # Add a cost based on the euclidean distance for each edge
    nx.set_edge_attributes(
        visibility_graph,
        {e: ((e[0][0] - e[1][0]) ** 2 + (e[0][1] - e[0][1]) ** 2) ** 0.5 for e in visibility_graph.edges()},
        "cost",
    )
    print(visibility_graph)
    return visibility_graph


@Utility.timing()
def a_star_path(start, end, G: nx.Graph):
    def dist(a, b):
        (x1, y1) = a
        (x2, y2) = b
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    return nx.astar_path(G, start, end, heuristic=dist, weight="cost")


@Utility.timing()
def dijkstra_path(start, end, G: nx.Graph):
    return nx.astar_path(G, start, end, weight="cost")


def test_connect_nearest_nodes():
    # Create an arbitrary graph
    G = nx.Graph()
    G.add_edges_from([(1, 2), (2, 3), (3, 4), (4, 1)])

    # Define the coordinates of the new node
    new_node_coords = (2.5, 2.5)

    # Find the nearest edge to the new node
    nearest_edge = None
    nearest_edge_dist = float("inf")
    for u, v in G.edges():
        edge = LineString([(u, v), G.nodes[u]["pos"], G.nodes[v]["pos"]])
        node = Point(new_node_coords)
        dist = edge.distance(node)
        if dist < nearest_edge_dist:
            nearest_edge = (u, v)
            nearest_edge_dist = dist

    # Calculate the distance between the new node and the nearest point on the nearest edge
    edge = LineString([nearest_edge, G.nodes[nearest_edge[0]]["pos"], G.nodes[nearest_edge[1]]["pos"]])
    node = Point(new_node_coords)
    nearest_point = edge.interpolate(edge.project(node))
    nearest_point_dist = nearest_point.distance(node)

    # Add the new node to the graph and connect it to the nearest point on the nearest edge
    # new_node = max(G.nodes) + 1
    G.add_node(new_node_coords, pos=new_node_coords)
    G.add_edge(new_node_coords, nearest_edge[0] if nearest_point_dist < 0.5 else nearest_edge[1])

    # Print the resulting graph
    print(G.nodes)


def dataset_test():

    dataset_name = "VM25"
    files = Utility.getAllCoverageFiles(dataset_name)
    for file_name in files:
        with open(file_name) as json_file:
            features = geojson.load(json_file)["features"]

        # Convert each feature to a Shapely geometry object
        geometries = {"obstacles": MultiPolygon(), "tasks": MultiLineString(), "boundary": Polygon()}
        for feature in features:
            if feature["geometry"]:
                geometries[feature["id"]] = shape(feature["geometry"])

        # Create a GeometryCollection with the geometries and their types
        G = naive_visibility_graph(geometries["boundary"], geometries["obstacles"], reduced_visibility=True)
        options = {"edgecolors": "tab:gray", "node_size": 50, "alpha": 0.7}
        # print(dijkstra_path((polygon.boundary.coords[0]), (holes[0].boundary.coords[0]), G))
        nx.draw_networkx_edges(G, nx.get_node_attributes(G, "pos"), width=1.0, alpha=0.5)
        nx.draw_networkx_nodes(G, nx.get_node_attributes(G, "pos"), node_color="tab:blue", **options)

        x, y = geometries["boundary"].exterior.xy
        plt.plot(x, y)
        for poly in geometries["obstacles"].geoms:
            xi, yi = poly.exterior.xy
            plt.plot(xi, yi)
        plt.show()
        plt.clf()


def convert_dataset_to_geojson():
    files = Utility.getAllCoverageFiles("AC300")

    for file_name in files:
        with open(file_name) as json_file:
            data = json.load(json_file)
        holes = []
        for obs in data["holes"]:
            hole = Polygon(obs)
            holes.append(hole)

        obstacles = MultiPolygon(holes)
        polygon = Polygon(data["polygon"])

        lines = []
        for line in data["lines"]:
            lines.append(LineString([line["start"], line["end"]]))

        tasks = MultiLineString(lines)
        feature_collection = geojson.FeatureCollection(
            [
                geojson.Feature(geometry=polygon.__geo_interface__, id="boundary"),
                geojson.Feature(geometry=obstacles.__geo_interface__, id="obstacles"),
                geojson.Feature(geometry=tasks.__geo_interface__, id="tasks"),
            ]
        )
        with open(file_name, "w") as f:
            geojson.dump(feature_collection, f)


if __name__ == "__main__":
    dataset_test()
if __name__ == "__main__":
    dataset_test()
if __name__ == "__main__":
    dataset_test()
