from setuptools import find_packages, setup

package_name = 'vision_dist_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs', [
            'docs/vision_dist_ctrl_spec.md',
            'docs/requirements_ja.md',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Vision-based forward/backward controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_distance_controller = vision_dist_ctrl.vision_distance_controller:main',
            'vision_cmd_logger = vision_dist_ctrl.vision_cmd_logger:main',
        ],
    },
)
