from setuptools import setup

package_name = 'locomotion_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name: ['gmr_mean_trajectory.csv'],
        package_name: ['gmm_params.npz'],
        package_name: ['safe_path_model_yaw.csv'],
        package_name: ['metrics_logger.py'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/wheel_control.launch.py',
        ]),
        ('share/' + package_name + '/docs', ['docs/generic_motion_control_spec.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_roboteq = locomotion_core.cmd_roboteq:main',
            'rover_velocity = locomotion_core.rover_velocity:main',
            'log_ctrl = locomotion_core.center_ctrl:main',
        ],
    },
)
