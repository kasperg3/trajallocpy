from setuptools import find_packages, setup

setup(
    name="trajallocpy",
    version="0.0.3",
    description="TrajAllocPy is a Python library that provides functionality for trajectory task Allocation using Consensus based bundle algorithm",
    long_description="",
    long_description_content_type="text/markdown",
    author="Kasper Rømer Grøntved",
    author_email="kaspergrontved@gmail.com",
    url="",
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[],
    extras_require={"test": ["pytest"]},
)
