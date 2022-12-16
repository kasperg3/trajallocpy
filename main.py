from task_allocation import Experiment, Utility, CoverageProblem

# import from csv
cp = Utility.loadCoverageProblem("data/OdenseSO.json")

exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)

# stage the experiment

# Solve

# run x

# add another agent
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import geopandas as gpd

polygon1 = Polygon(
    [
        (0, 5),
        (1, 1),
        (3, 0),
    ]
)

p = gpd.GeoSeries(polygon1)
p.plot()
plt.show()

exp.run()
