from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['opening_door'],  
    package_dir={'': 'src'},
    scripts=['src/open_the_door_main.py', 'src/open_door_smach.py']
)

setup(**d)  