from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup

pkg_config = generate_distutils_setup(
    packages=['multirobotsimulations'],
    scripts=[],
    package_dir={'': 'scripts'}
)

setup(**pkg_config)