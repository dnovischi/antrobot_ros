import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'antrobot_ros'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the config folder and its contents
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'docs'), glob(os.path.join('docs', '*.md'))),
        # Include evaluation scripts
        (os.path.join('share', package_name, 'scripts', 'evaluation', 'odom'), glob(os.path.join('scripts', 'evaluation', 'odom', '*.sh'))),
        # Include diagnostic scripts
        (os.path.join('share', package_name, 'scripts', 'diagnostic'), glob(os.path.join('scripts', 'diagnostic', '*.py'))),
        
    ],
    install_requires=['setuptools', 'smbus2', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='Dan Novischi',
    maintainer_email='dan_marius.novischi@upb.ro',
    description='This package provides ROS2 nodes and interfaces to control the Antrobot.',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rdrive_node = antrobot_ros.rdrive_node:main',
            'keyboard_teleoperation_node = antrobot_ros.keyboard_teleoperation_node:main',
            'laserscan_to_pointcloud_node = antrobot_ros.laserscan_to_pointcloud_node:main',
            'joint_state_estimator_node = antrobot_ros.joint_state_estimator_node:main',
            'odom_eval_node = antrobot_ros.odom_eval_node:main',
            'odom_monitor = antrobot_ros.odom_monitor:main',
            'explore_lite_map_converter = antrobot_ros.explore_lite_map_converter:main',
            'quick_odom_monitor = scripts.diagnostic.quick_odom_monitor:main',
            'tf_timing_capture = scripts.diagnostic.tf_timing_capture:main',
            'tf_timing_diagnostic = scripts.diagnostic.tf_timing_diagnostic:main'
        ],
    },
    
    # package_data={
    #     package_name: ['config/antrobot_params.yaml','config/keyboard_teleoperation_params.yaml'],
    # },
    # scripts=[
    #     'antrobot_ros/rdrive_node.py',
    #     'antrobot_ros/keyboard_teleoperation_node.py'
    # ],
)
