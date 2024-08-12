import json
import random

import contextily as cx
import geojson
import matplotlib.pyplot as plt
import shapely
import trajgenpy.Logging
from trajgenpy import Utils
from trajgenpy.Geometries import (
    GeoMultiPolygon,
    GeoMultiTrajectory,
    GeoPolygon,
    decompose_polygon,
    generate_sweep_pattern,
    get_sweep_offset,
)
from trajgenpy.Query import query_features

log = trajgenpy.Logging.get_logger()


def get_line_segments(polygon: GeoPolygon):
    polygon_list = decompose_polygon(polygon.get_geometry())

    log.info(f"Number of polygons: {len(polygon_list)}")

    offset = get_sweep_offset(0.1, 15, 90)
    result = []
    for decomposed_poly in polygon_list:
        sweeps_connected = generate_sweep_pattern(decomposed_poly, offset, connect_sweeps=False)
        result.extend(sweeps_connected)
    return result


def export_to_geojson(tasks: GeoMultiTrajectory, polygon: GeoPolygon):
    log.info(f"Number of sweeps: {len(tasks.get_geometry().geoms)}")
    geojson_collection = geojson.FeatureCollection(
        [
            polygon.to_geojson(id="boundary"),
            multi_traj.to_geojson(id="tasks"),
        ]
    )
    with open("environment.geojson", "w") as f:
        geojson.dump(geojson_collection, f)


import osmnx as ox

if __name__ == "__main__":
    # polygon_file = "DemaScenarios/FlatTerrainNature.geojson"
    # polygon_file = "DemaScenarios/HillyTerrainNature.geojson"
    # polygon_file = "DemaScenarios/Urban.geojson"
    polygon_file = "DemaScenarios/Water.geojson"

    # Load the GeoJSON data
    with open(polygon_file, "r") as f:
        data = json.load(f)

    # Extract the polygon coordinates
    coordinates = data["features"][0]["geometry"]["coordinates"][0]

    # Create the shapely Polygon object
    polygon = GeoPolygon(shapely.Polygon(coordinates)).set_crs("EPSG:4093")
    polygon.plot(facecolor="none", edgecolor="black", linewidth=2)

    query_polygon = GeoPolygon(shapely.Polygon(coordinates)).set_crs("WGS84")
    # features = query_features(
    #     query_polygon,
    #     {
    #         "natural": ["coastline"],
    #     },
    # )
    # GeoMultiTrajectory(features["natural"]).set_crs("EPSG:4093").plot(color="blue", linewidth=2)

    tasks = get_line_segments(polygon)

    multi_traj = GeoMultiTrajectory(tasks, "EPSG:4093")
    multi_traj.plot(color="red")

    Utils.plot_basemap(provider=cx.providers.OpenStreetMap.Mapnik, crs="EPSG:4093")
    export_to_geojson(multi_traj, polygon)

    # No axis on the plot
    plt.axis("equal")
    plt.axis("off")
    plt.show()
