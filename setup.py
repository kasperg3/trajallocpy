"""Compatibility shim. Project metadata and dependencies live in
``pyproject.toml`` (PEP 621); this only exists for very old pip/build tooling."""

from setuptools import setup

setup()
