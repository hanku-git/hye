from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hanyang_matching_open3d'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BP',
    maintainer_email='bp@hanyang.ac.kr',
    description='Open3D-based template matching node for bin picking system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'matching_node = hanyang_matching_open3d.nodes.matching_node:main',
            'matching_node_full = hanyang_matching_open3d.nodes.matching_node_full:main',
        ],
    },
)

