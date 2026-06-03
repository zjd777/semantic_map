import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'semantic_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='2436210442@qq.com',
    description='Semantic mapping and semantic navigation tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_voxel_mapper = semantic_mapping.semantic_voxel_mapper:main',
            'semantic_map_marker_publisher = semantic_mapping.semantic_map_marker_publisher:main',
            'semantic_map_editor = semantic_mapping.semantic_map_editor:main',
            'semantic_occupancy_grid_saver = semantic_mapping.occupancy_grid_saver:main',
            'semantic_navigation_tool = semantic_mapping.semantic_navigation_tool:main',
        ],
    },
)
