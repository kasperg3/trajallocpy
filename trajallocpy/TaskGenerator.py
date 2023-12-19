from functools import partial

import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import osmnx as ox
import shapely
from pyproj import CRS, Transformer


def main():
    # Download the water features data within the bounding box
    tags = {
        "natural": ["water", "wetland"],
        "landuse": ["farmland", "grassland"],
        # "highway": ["primary", "secondary", "tertiary", "unclassified", "road"],
    }

    polygon = shapely.Polygon([(10.490913, 55.315346), (10.576744, 55.315346), (10.576744, 55.337417), (10.490913, 55.337417)])
    geometries = ox.features_from_polygon(polygon, tags=tags)

    # Convert individual polygons to a MultiPolygon
    multi_polygon = geometries.geometry.unary_union

    # Create a GeoDataFrame from the MultiPolygon
    gdf = gpd.GeoDataFrame(geometry=[shapely.intersection(polygon, multi_polygon)], crs=geometries.crs)
    gdf = gdf.to_crs("EPSG:2197")

    gdf_polygon = gpd.GeoDataFrame(geometry=[polygon], crs=geometries.crs)
    gdf_polygon = gdf_polygon.to_crs("EPSG:2197")
    # Get the minimum x and y coordinate values
    min_x = gdf.bounds["minx"].min()
    min_y = gdf.bounds["miny"].min()

    # Apply the normalization to the geometry column of the GeoDataFrame
    gdf["geometry"] = gdf["geometry"].translate(xoff=-min_x, yoff=-min_y)
    gdf_polygon["geometry"] = gdf_polygon["geometry"].translate(xoff=-min_x, yoff=-min_y)

    print(len(list(gdf.geometry[0].geoms)))
    # Plot the lakes
    ax = gdf_polygon.plot(facecolor="white", edgecolor="red")
    ax = gdf.plot(ax=ax, facecolor="white", edgecolor="black")
    # ax.set_axis_off()
    plt.show()


if __name__ == "__main__":
    main()
