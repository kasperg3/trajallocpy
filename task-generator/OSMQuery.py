import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import osmnx as ox
import shapely


def main():
    # Download the water features data within the bounding box
    tags = {
        "natural": ["water"],
        # "landuse": ["farmland", "grassland"],
        # "highway": ["primary", "secondary", "tertiary", "unclassified", "road"],
    }
    polygon = shapely.Polygon([(55.337417, 10.490913), (55.338588, 10.576744), (55.315346, 10.570908), (55.313001, 10.488510)]).reverse()
    print("valid poly: " + str(polygon.is_valid))
    north = 55.337417
    south = 55.315346
    west = 10.490913
    east = 10.576744
    # geometries = ox.features_from_polygon(polygon, tags=tags)
    geometries = ox.features_from_bbox(north=north, south=south, east=east, west=west, tags=tags)
    # Convert individual polygons to a MultiPolygon
    multi_polygon = geometries.geometry.unary_union

    # Create a GeoDataFrame from the MultiPolygon
    lake_gdf = gpd.GeoDataFrame(geometry=[multi_polygon], crs=geometries.crs)

    # Plot the lakes
    ax = lake_gdf.plot(facecolor="blue", edgecolor="black")
    ax.set_axis_off()
    plt.show()


if __name__ == "__main__":
    main()
