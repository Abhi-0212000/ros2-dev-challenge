from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_wave_pkg'

from generate_parameter_library_py.setup_helper import generate_parameter_module
generate_parameter_module(
    "sine_wave_pub_params",  # The name of the generated Python module
    "config/publisher_params.yaml"  # The path to your YAML file
)

generate_parameter_module(
    "sine_wave_sub_params",  # The name of the generated Python module
    "config/subscriber_params.yaml"  # The path to your YAML file
)

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'test_images'),
         glob('test/test_images/*.jpg')),
    ],
    install_requires=['setuptools',
                     'numpy',
                     'matplotlib'  # Adding matplotlib here since it's a pip package
    ],
    zip_safe=True,
    maintainer='Abhishek Nannuri',
    maintainer_email='abhishek.nannuri@outlook.com',
    description='ROS2 package for sine wave publishing and visualization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sine_wave_publisher = ros2_wave_pkg.sine_wave_publisher_node:main',
            'sine_wave_subscriber = ros2_wave_pkg.sine_wave_subscriber_node:main'
        ],
    },
)