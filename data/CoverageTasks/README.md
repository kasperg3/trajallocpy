<h1 align="center">Area Coverage task dataset for multi-robot task allocation 👋</h1>
<p>
  <img alt="Version" src="https://img.shields.io/badge/version-0.0.1-blue.svg?cacheSeconds=2592000" />
  <a href="https://doi.org/10.5281/zenodo.7763104"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.7763104.svg" alt="DOI"></a>
  <a href="https://github.com/kefranabg/readme-md-generator/graphs/commit-activity" target="_blank">
    <img alt="Maintenance" src="https://img.shields.io/badge/Maintained%3F-yes-green.svg" />
  </a>
  <a href="https://github.com/kasperg3/swarm-simulator/blob/79fbc5c29036169ec56d4c07bd64e2df01b3bf38/LICENCE" target="_blank">
    <img alt="License: MIT" src="https://img.shields.io/github/license/kasperg3/CoverageTasks" />
  </a>
  <!-- <a href=" " target="_blank">
    <img alt="Build" src="https://github.com/kasperg3/swarm-simulator/actions/workflows/build.yml/badge.svg" />
  </a> -->
</p>

![AC300 task example](https://github.com/kasperg3/CoverageTasks/blob/498c7cf2e65521f9a4e9524781397434f7d96bd5/.assets/AC300AC70008tasks.png)

# Coverage task dataset
This repository contains the collection of dataset for the area coverage problem. Each problem is contained within a single json file to easily import it into your favorite programming language. 

The problems contains a bounding polygon representing the coverage area, a set of holes represented as a list of coordinates, and all problems contains predefined coverage tasks generated using existing routes from https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset. 

## The datasets
The datasets are generated from existing problems datasets provided by: 
* VM25 - I. Vandermeulen, R. Groß, and A. Kolling, “Turn-minimizing multirobot coverage,” IEEE International Conference on Robotics and Automation (ICRA), 2019, pp. 1014–1020.
* AC300 - https://github.com/ethz-asl/polygon_coverage_planning
* H2 - W. H. Huang, “Optimal line-sweep-based decompositions for coverage algorithms,” IEEE International Conference on Robotics and Automation (ICRA), 2001

### The format
The format of the datasets provided in this repository is following the [RFC-7946](https://www.rfc-editor.org/rfc/rfc7946) standard called GeoJSON. GeoJson is a videly used geospatial data interchange format based on the json format. It defines several types of JSON objects and
the manner in which they are combined to represent data about
geographic features, their properties, and their spatial extents.

All the coverage problems are defined as a FeatureCollection and contains Features such as a boundary (Polygon), a list of coverage tasks (MultiLineString), and can contain obstacles defines as a set of polygons (MultiPolygon).


## Using the Datasets
Using python to load a json file with the geojson format can be done as follows: 
```python
import geojson
with open(path_to_file) as f:
    gj = geojson.load(f)
features = gj['features']
```



# Author

👤 **Kasper Andreas Rømer Grøntved**

* Website: https://blog.grontved.dk
* GitHub: [@kasperg3](https://github.com/kasperg3)


# Show your support

Give a ⭐️ if this project helped you!

# 📝 License

Copyright © 2023 [Kasper Andreas Rømer Grøntved](https://github.com/kasperg3).<br />
This project is [MIT](https://github.com/kasperg3/CoverageTasks/blob/main/LICENCE) licensed.
