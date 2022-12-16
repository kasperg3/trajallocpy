from task_allocation import Experiment, Utility, CoverageProblem

# import from csv
cp = Utility.loadCoverageProblem("data/OdenseSO.json")

exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)

# stage the experiment

# Solve

# run x

# add another agent


exp.run()
