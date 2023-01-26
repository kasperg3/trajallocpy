from task_allocation import Experiment, Utility
import numpy as np

np.random.seed(123423)
# import from csv
cp = Utility.loadCoverageProblem("data/OdenseSO.json", nr=3)


# stage the experiment
exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)

# Solve

# run x

# add another agent

exp.solve()
