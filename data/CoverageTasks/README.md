<h1 align="center">Area Coverage task dataset for multi-robot task allocation 👋</h1>
<p>
  <img alt="Version" src="https://img.shields.io/badge/version-0.0.1-blue.svg?cacheSeconds=2592000" />
  <a href="https://zenodo.org/badge/latestdoi/606742253"><img src="https://zenodo.org/badge/606742253.svg" alt="DOI"></a>
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

## The format
The json file is structured as follows:
```
Polygon: [Vertex, ...]
Holes: [Hole, ...]
Tasks: [Task, ...]
```


## The datasets

* VM25 - I. Vandermeulen, R. Groß, and A. Kolling, “Turn-minimizing multirobot coverage,” IEEE International Conference on Robotics and Automation (ICRA), 2019, pp. 1014–1020.
* AC300 - https://github.com/ethz-asl/polygon_coverage_planning
* H2 - W. H. Huang, “Optimal line-sweep-based decompositions for coverage algorithms,” IEEE International Conference on Robotics and Automation (ICRA), 2001

# Author

👤 **Kasper Andreas Rømer Grøntved**

* Website: https://blog.grontved.dk
* GitHub: [@kasperg3](https://github.com/kasperg3)


# Show your support

Give a ⭐️ if this project helped you!

# 📝 License

Copyright © 2023 [Kasper Andreas Rømer Grøntved](https://github.com/kasperg3).<br />
This project is [MIT](https://github.com/kasperg3/CoverageTasks/blob/main/LICENCE) licensed.
