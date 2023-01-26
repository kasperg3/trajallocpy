from task_allocation import Experiment, Utility

# import from csv
cp = Utility.loadCoverageProblem("data/OdenseSO.json", nr=3)


# stage the experiment
exp = Experiment.runner(coverage_problem=cp, enable_plotting=True)

# Solve

# run x

# add another agent

exp.run()
