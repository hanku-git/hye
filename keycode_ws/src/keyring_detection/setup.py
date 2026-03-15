from setuptools import find_packages, setup

package_name = 'keyring_detection'

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
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 package for detecting red dot and keyring in Zivid camera images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_red_dot_node_for_drum = keyring_detection.detect_red_dot_node_for_drum:main',
            'detect_red_dot_node_for_holder = keyring_detection.detect_red_dot_node_for_holder:main',
            'detect_red_dot_node_for_holder_alt = keyring_detection.detect_red_dot_node_for_holder_alt:main',
        ],
    },
)
