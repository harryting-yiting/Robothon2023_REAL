from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['gripper','scservo_sdk', 'RS485Gripper'],
    package_dir={'': 'src'}
)
setup(**d)