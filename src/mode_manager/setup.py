from glob import glob
from setuptools import find_packages, setup

setup(
    name='mode_manager', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mode_manager']),
        ('share/mode_manager', ['package.xml']),
        ('share/mode_manager/config', glob('config/*.yaml')),
        ('share/mode_manager/launch', glob('launch/*.launch.py')),
        ('share/mode_manager/doc', glob('doc/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='user', maintainer_email='user@example.com',
    description='UWB and Vision docking mode manager', license='Apache-2.0',
    entry_points={'console_scripts': ['mode_manager = mode_manager.node:main']},
)
