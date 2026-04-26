import os
from glob import glob
from setuptools import setup

package_name = 'opp_predictor'

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
        # Maps + bound files: install whole subtrees so the node can find them at runtime
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ] + [
        # csv/<map_name>/* gets one entry per subdir
        (os.path.join('share', package_name, 'csv', os.path.basename(d)),
         glob(os.path.join(d, '*')))
        for d in glob('csv/*') if os.path.isdir(d)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang50-guaidao',
    maintainer_email='yang50@seas.upenn.edu',
    description='LiDAR-based opponent detector for f1tenth real-car racing.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opp_predictor_node = opp_predictor.opp_predictor_node:main',
        ],
    },
)
