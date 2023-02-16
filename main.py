from task_allocation import Experiment, Utility
import numpy as np

seed = 8986413
np.random.seed(seed)

# import from csv
cp = Utility.loadCoverageProblem("data/SDUAreaCoverage/OdenseSO.json", nr=3)

# stage the experiment
for i in range(1):
    exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)
    exp.solve()

# Make sure to document the following datapoints:
# Dataset Name,Num of Vertices,Area,Num of Non-Convex Vertices,MSA- Inital decomposition,Num of Initial Cells,MSA-Final Decomposition,Num of final cells,Sum of Costs of Routes,Length of Service Tracks,Compute Time-Cell Decomposition,Compute Time-Service Tracks,Compute Time-Routing,Total Compute Time
