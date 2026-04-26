import os
from glob import glob
from setuptools import setup

package_name = 'opp_dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang50-guaidao',
    maintainer_email='yang50@seas.upenn.edu',
    description='Autonomous opponent driver for f1tenth_gym_ros 2-car mode.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opp_dummy_node = opp_dummy.opp_dummy_node:main',
        ],
    },
)
