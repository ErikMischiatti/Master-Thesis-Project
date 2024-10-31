from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

print("Calling setup.py")

setup_args = generate_distutils_setup(
    packages=['dhb_ros'],
    package_dir={'': 'src'}
)
setup(**setup_args)
