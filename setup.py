#!/usr/bin/env python3
"""
Setup script for 3D Cable-Driven Positioning System - Python Host

This setup script allows for easy installation of the cable system
and its dependencies.
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the contents of README file
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

# Read requirements from requirements.txt
requirements = []
requirements_file = this_directory / "requirements.txt"
if requirements_file.exists():
    with open(requirements_file) as f:
        requirements = [
            line.strip() 
            for line in f 
            if line.strip() and not line.startswith('#')
        ]

setup(
    name="cable-positioning-system",
    version="0.1.0",
    author="Cable System Project",
    author_email="",
    description="3D Cable-Driven Positioning System - Python Host",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Mathematics",
        "Topic :: Scientific/Engineering :: Physics", 
        "Topic :: System :: Hardware :: Hardware Drivers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=6.2.0",
            "pytest-asyncio>=0.20.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
        ],
        "visualization": [
            "matplotlib>=3.5.0",
            "plotly>=5.0.0",
        ],
        "web": [
            "fastapi>=0.70.0",
            "uvicorn>=0.15.0",
            "websockets>=10.0",
        ],
        "advanced": [
            "scipy>=1.7.0",
            "psutil>=5.8.0",
        ]
    },
    entry_points={
        "console_scripts": [
            "cable-system=main:main",
        ],
    },
    include_package_data=True,
    package_data={
        "cable_system": ["config/*.yaml"],
    },
    zip_safe=False,
) 