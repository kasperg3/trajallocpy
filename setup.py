from setuptools import find_packages, setup

with open("README.md") as f:
    readme = f.read()

with open("LICENSE") as f:
    license = f.read()

setup(
    name="TaskAllocation",
    version="0.0.1",
    description="A collection of task allocation algorithms implemented in python",
    long_description=readme,
    author="Kasper Grøntved",
    author_email="kaspergrontved@gmail.com",
    url="https://github.com/kasperg3",
    license=license,
    packages=find_packages(exclude=("tests", "docs")),
)
