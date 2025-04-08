from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ensure launch files are found under custom_explorer package
        (os.path.join('share', package_name, 'launch_file'), glob('launch_file/*.launch.py')),
        # ensure config files are found under custom_explorer package
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izen',
    maintainer_email='e1406600@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "missionStart = mission_control.mission_control:main",
        ],
    },
)
