from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'slam_offline_pgo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register with ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stepan Letsko',
    maintainer_email='stepanletsko5@gmail.com',
    description='Offline loop closure and pose graph optimisation for FAST-LIO2',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Stage 1: read bag, extract keyframes
            'io_bag       = slam_offline_pgo.io_bag:main',
            # Stage 2: run the full offline PGO pipeline
            'pgo_offline  = slam_offline_pgo.pgo_offline:main',
            # Utilities
            'map_renderer = slam_offline_pgo.map_renderer:main',
            'evaluate     = slam_offline_pgo.evaluate:main',
            'visualise_sc = slam_offline_pgo.visualise_sc:main',
        ],
    },
)
