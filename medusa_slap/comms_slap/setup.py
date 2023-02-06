#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['comms_slap_algorithms', 'comms_slap_ros'],
 package_dir={'comms_slap_algorithms': 'src/comms_slap_algorithms', 'comms_slap_ros': 'src/comms_slap_ros'}
)

setup(**d)
