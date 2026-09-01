from glob import glob

from setuptools import find_packages, setup

package_name = 'aruco_distance_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/distance_publisher', [
            *glob('aruco_distance_publisher/distance_publisher/calib_result*.npz'),
        ]),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Aruco marker detection and distance publishing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_distance_publisher = aruco_distance_publisher.distance_publisher.aruco_distance_publisher_node:main',
        ],
    },
)
