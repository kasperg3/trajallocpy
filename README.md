



# TrajAllocPy: Decentralized Multi-Robot Task Allocation for Area Coverage

![License](https://img.shields.io/badge/license-MIT-blue.svg)
[![Build Status](https://github.com/kasperg3/TrajAllocPy/actions/workflows/test.yml/badge.svg)](https://github.com/kasperg3/TrajAllocPy/actions/workflows/test.yml)
[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)

<p align="center">
  <img src="https://github.com/kasperg3/trajallocpy/blob/e635b71b0950f02abd04f8221ceec85ef949c46d/.assets/trajectory_allocation.gif" />
</p>

**TrajAllocPy** is a Python library for decentralized multi-robot task allocation using the **Consensus Based Bundle Algorithm (CBBA)**. It's designed for area coverage problems where multiple robots need to efficiently distribute and execute trajectory-based tasks while avoiding obstacles.

## 🚀 Key Features

- **Decentralized Algorithm**: Uses CBBA for distributed task allocation without central coordination
- **Obstacle Avoidance**: Built-in support for complex environments with obstacles
- **Trajectory Tasks**: Handles line-segment based coverage tasks (e.g., search patterns, surveillance routes)
- **Visualization**: Real-time plotting of robot paths and task allocation
- **Flexible Configuration**: Customizable robot capacities, velocities, and task rewards
- **GeoJSON Support**: Load real-world environments from GeoJSON files

## 📦 Installation

### Quick Install
```bash
pip install trajallocpy
```

### Development Install
```bash
git clone https://github.com/kasperg3/TrajAllocPy.git
cd TrajAllocPy
pip install -r requirements.txt
pip install -e .
```

## 🎯 Quick Start

Here's a simple example to get you started:

```python
import numpy as np
from trajallocpy import Agent, CoverageProblem, Experiment, Task
import shapely

# Set random seed for reproducible results
np.random.seed(123)

# Create a simple environment
boundary = shapely.geometry.Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])
obstacles = shapely.geometry.MultiPolygon([
    shapely.geometry.Polygon([(20, 20), (40, 20), (40, 40), (20, 40)]),
    shapely.geometry.Polygon([(60, 60), (80, 60), (80, 80), (60, 80)])
])

# Define tasks (trajectory segments to cover)
tasks = [
    Task.TrajectoryTask(0, shapely.geometry.LineString([(10, 10), (15, 15)]), reward=100),
    Task.TrajectoryTask(1, shapely.geometry.LineString([(50, 30), (70, 35)]), reward=100),
    Task.TrajectoryTask(2, shapely.geometry.LineString([(30, 70), (45, 85)]), reward=100),
]

# Create coverage problem
coverage_problem = CoverageProblem.CoverageProblem(
    restricted_areas=obstacles,
    search_area=boundary,
    tasks=tasks
)

# Create agents
agents = [
    Agent.config(0, [(10, 50)], capacity=500, max_velocity=10),
    Agent.config(1, [(50, 10)], capacity=500, max_velocity=10),
    Agent.config(2, [(90, 90)], capacity=500, max_velocity=10),
]

# Run the experiment
experiment = Experiment.Runner(
    coverage_problem=coverage_problem,
    enable_plotting=True,  # Shows real-time visualization
    agents=agents
)

# Solve and get results
experiment.solve()
compute_time, iterations, path_lengths, task_lengths, path_costs, rewards, routes, max_cost = experiment.evaluateSolution()
print(f"Computation time: {compute_time:.3f} seconds")
print(f"Total route length: {sum(path_lengths.values()):.2f}")
print(f"Total rewards: {sum(rewards.values()):.2f}")
```

## 📚 Complete Example

For a comprehensive example, see [`example_simple.py`](example_simple.py) which includes:
- Creating environments programmatically
- Loading from GeoJSON files
- Configuring multiple agents
- Visualizing results
- Performance evaluation

Run the example:
```bash
python example_simple.py
```

## 🗺️ Working with GeoJSON

TrajAllocPy supports loading environments from GeoJSON files. Your GeoJSON should contain features with these IDs:
- `"boundary"`: The search area boundary (Polygon)
- `"obstacles"`: Restricted areas to avoid (MultiPolygon)  
- `"tasks"`: Trajectory segments to cover (MultiLineString)

```python
from trajallocpy import Agent, CoverageProblem, Experiment, Task
import geojson
from shapely import geometry

# Load environment from GeoJSON
with open("environment.geojson") as f:
    geojson_data = geojson.load(f)

# Extract geometries
geometries = {}
for feature in geojson_data["features"]:
    if feature["geometry"]:
        geometries[feature["id"]] = geometry.shape(feature["geometry"])

# Create tasks from the loaded geometries
tasks = [
    Task.TrajectoryTask(i, task_geom, reward=100) 
    for i, task_geom in enumerate(geometries["tasks"].geoms)
]

# Continue with coverage problem setup...
```

## 🔧 Configuration Options

### Agent Configuration
```python
agent = Agent.config(
    0,                             # Unique agent identifier
    [(0, 0)],                      # Starting position [x, y]
    1000,                          # Maximum travel distance
    max_velocity=10                # Maximum speed
)
```

### Task Configuration
```python
task = Task.TrajectoryTask(
    id=0,                          # Unique task identifier
    trajectory=line_geometry,      # Shapely LineString
    reward=100                     # Task completion reward
)
```

### Coverage Problem
```python
problem = CoverageProblem.CoverageProblem(
    restricted_areas=obstacles,    # Areas to avoid (MultiPolygon)
    search_area=boundary,          # Valid operation area (Polygon)
    tasks=task_list               # List of tasks to allocate
)
```

## 📊 Understanding Results

The `evaluateSolution()` method returns a tuple with:
1. **Computation time** (seconds)
2. **Algorithm iterations**
3. **Path lengths** (dictionary: agent_id -> total path length)
4. **Task lengths** (dictionary: agent_id -> total task length covered)
5. **Path costs** (dictionary: agent_id -> total travel cost)
6. **Rewards** (dictionary: agent_id -> total rewards collected)
7. **Route assignments** (dictionary: agent_id -> coordinate list of full route)
8. **Maximum route cost** (bottleneck agent cost)

## 🎨 Visualization

Enable real-time visualization by setting `enable_plotting=True` in the experiment runner. This shows:
- Environment boundaries and obstacles
- Task locations and assignments
- Agent paths and positions
- Real-time algorithm progress

## 🏗️ Architecture

TrajAllocPy implements the CBBA algorithm with these key components:

- **Agent**: Individual robot with position, capacity, and capabilities
- **Task**: Trajectory segment with location, reward, and requirements  
- **CoverageProblem**: Environment definition with boundaries and obstacles
- **Experiment.Runner**: Algorithm execution and visualization manager

## 📈 Performance Tips

- **Task Density**: More tasks than agents generally leads to better solutions
- **Agent Capacity**: Higher capacity allows longer routes but may reduce parallelism
- **Environment Complexity**: Simple convex environments solve faster
- **Visualization**: Disable plotting (`enable_plotting=False`) for faster computation

## 🔬 Research & Citations

This library implements the research presented in:

```bibtex
@inproceedings{grontved2023icar,
  title={Decentralized Multi-UAV Trajectory Task Allocation in Search and Rescue Applications},
  author={Gr{\o}ntved, Kasper Andreas R{\o}mer and Schultz, Ulrik Pagh and Christensen, Anders Lyhne},
  booktitle={21st International Conference on Advanced Robotics},
  year={2023},
  organization={IEEE}
}
```

## 🤝 Contributing

We welcome contributions! Please see:
- [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines
- [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) for community standards
- Open issues for bug reports and feature requests

### Development Setup
```bash
git clone https://github.com/kasperg3/TrajAllocPy.git
cd TrajAllocPy
pip install -e .
python -m pytest tests/  # Run tests
```

## 📄 License

This project is licensed under the [MIT License](LICENSE).

## 🆘 Support

- **Issues**: [GitHub Issues](https://github.com/kasperg3/TrajAllocPy/issues)
- **Email**: [kaspergrontved@gmail.com](mailto:kaspergrontved@gmail.com)
- **Documentation**: See examples and docstrings for detailed API usage

## 🔗 Related Projects

- [CoverageTasks](https://github.com/kasperg3/CoverageTasks): Dataset format for coverage task problems
- [Coverage Tasks Dataset](https://github.com/kasperg3/CoverageTasks): Benchmark problems for evaluation

---

**TrajAllocPy** - Efficient, decentralized task allocation for autonomous robot teams! 🤖✨
