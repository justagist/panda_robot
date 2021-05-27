#!/usr/bin/env python
import os, sys
from setuptools import setup

# The text of the README file
README = open(os.path.join(os.path.dirname(__file__), "README.md")).read()

try:  # -- horrible!!
    from catkin_pkg.python_setup import generate_distutils_setup
    d = generate_distutils_setup()
except:
    d = {}

d['name'] = "panda_robot"
d['packages'] = ['panda_robot', 'panda_robot.utils']
d['version'] = "0.0.5"
d['package_dir'] = {'': 'src'}
d['long_description_content_type'] = "text/markdown"
d['long_description'] = README
d['url'] = "https://justagist.github.io/panda_robot"
d['project_urls'] = {
    "Bug Tracker": "https://github.com/justagist/panda_robot/issues",
    "Documentation": "https://justagist.github.io/panda_robot/DOC",
    "Source Code": "https://github.com/justagist/panda_robot",
}
if sys.version_info > (3,5):
    d['install_requires'] = ['numpy>=1.20.3', 'numpy-quaternion==2021.4.5.14.42.35']
else:
    d['install_requires'] = ['numpy>=1.16.6', 'numpy-quaternion==2021.4.5.14.42.35']
d['description'] = "Unified ROS-Python API for Franka Emika Panda robot using Franka ROS Interface"
d['author'] = "Saif Sidhik"
d['author_email'] = "sxs1412@bham.ac.uk"
d['license'] = "Apache 2.0"
d['classifiers'] = [
    "License :: OSI Approved :: Apache Software License",
    "Programming Language :: Python :: 2.7",
    "Programming Language :: Python :: 3.6",
    "Programming Language :: Python :: 3.7",
    "Programming Language :: Python :: 3.8",
    "Topic :: Software Development :: Libraries :: Python Modules",
    "Framework :: Robot Framework :: Library",
]

setup(**d)
