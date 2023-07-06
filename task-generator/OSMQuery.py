import geopandas as gpd
import matplotlib.pyplot as plt
import networkx as nx
import osmnx as ox
import shapely


def main():
    # Download the water features data within the bounding box
    tags = {
        "natural": ["water"],
        "landuse": ["farmland", "grassland"],
        # "highway": ["primary", "secondary", "tertiary", "unclassified", "road"],
    }

    polygon = shapely.Polygon([(10.490913, 55.315346), (10.576744, 55.315346), (10.576744, 55.337417), (10.490913, 55.337417)])
    geometries = ox.features_from_polygon(polygon, tags=tags)
    # Convert individual polygons to a MultiPolygon
    multi_polygon = geometries.geometry.unary_union

    # TODO group the different categories by type

    # Generate appropriate tasks for each geometry

    #

    # Create a GeoDataFrame from the MultiPolygon
    gdf = gpd.GeoDataFrame(geometry=[multi_polygon], crs=geometries.crs)
    len(list(gdf.geometry[0].geoms))
    # Plot the lakes
    ax = gdf.plot(facecolor="blue", edgecolor="black")
    ax.set_axis_off()
    plt.show()


if __name__ == "__main__":
    main()
