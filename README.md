



# TrajAllocPy: Decentralized multi-robot/agent task allocation for area coverage problems

![License](https://img.shields.io/badge/license-MIT-blue.svg)
[![Build Status](https://github.com/kasperg3/TrajAllocPy/actions/workflows/test.yml/badge.svg)](https://github.com/kasperg3/TrajAllocPy/actions/workflows/test.yml)
[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)

<p align="center">
  <img src="https://github.com/kasperg3/trajallocpy/blob/e635b71b0950f02abd04f8221ceec85ef949c46d/.assets/trajectory_allocation.gif" />
</p>

**TrajAllocPy** is a Python library that provides functionality for trajectory task allocation based on CBBA (**C**onsensus **B**ased **B**undle **A**lgorithm).


## Coverage task dataset
This repository uses dataset format given in: https://github.com/kasperg3/CoverageTasks 

## Installation

The package is regularly updated and new releases are created when significant changes to the main branch has happened.

```bash
pip install trajallocpy
```

## Build from source
You can install TrajAllocPy using `pip`. Simply navigate to your projects root directory and run:

```bash
pip install -r requirements.txt
pip install -e .
```

The "-e" argument for pip is to enable the developer to edit the python source code and perform tests without having to rebuild everything.

## Usage

To use TrajAllocPy in your Python project, you can import it as follows:

```python
import TrajAllocPy
```

You can then use the provided functions and classes to perform trajectory allocaion and task planning based on the consensus based bundle algorithm algorithm.
See the examples in [exmaples.md](.assets/example.md)

# Contributing & Development

Install the bindings in dev mode:

```bash
pip install -e .
```

To contribute to TrajAllocPy, start by forking the repository on GitHub. Create a new branch for your changes, make the necessary code edits, commit your changes with clear messages, and push them to your fork. Create a pull request from your branch to the original repository, describing your changes and addressing any related issues. Once your pull request is approved, a project maintainer will merge it into the main branch.

## Citation

If you use TrajAllocPy in your work, please cite the following paper:

```bibtex
@inproceedings{grontved2022icar,
  title={Decentralized Multi-UAV Trajectory Task Allocation in Search and Rescue Applications},
  author={Gr{\o}ntved, Kasper Andreas R{\o}mer and Schultz, Ulrik Pagh and Christensen, Anders Lyhne},
  booktitle={21st International Conference on Advanced Robotics},
  year={2023},
  organization={IEEE}
}
```

## License

This library is released under the [MIT License](LICENSE). Feel free to use, modify, and distribute it in your projects.

## Issues and Contributions

If you encounter any issues or have ideas for improvements, please open an issue on the [GitHub repository](https://github.com/kasperg3/TrajAllocPy). Contributions in the form of pull requests are also welcome.

## Support

For support and inquiries, you can contact the maintainers of this library at [kaspergrontved@gmail.com](mailto:kaspergrontved@gmail.com).

Thank you for using TrajAllocPy! We hope it proves to be a valuable tool for your trajectory generation and task planning needs.
