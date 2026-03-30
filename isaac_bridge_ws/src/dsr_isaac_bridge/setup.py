from setuptools import find_packages, setup

package_name = 'dsr_isaac_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanku',
    maintainer_email='hanku@kshan',
    description='Bridge between Doosan ROS2 services and Isaac Sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = dsr_isaac_bridge.bridge_node:main',
        ],
    },
)
