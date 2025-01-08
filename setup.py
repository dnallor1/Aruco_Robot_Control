from setuptools import find_packages, setup

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_control.launch.py']),  # Example for launch files
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),  # Example for config files
    ],
    install_requires=[
        'setuptools',
        'opencv-python',  
        'numpy',          
    ],
    zip_safe=True,
    maintainer='Ruman Mondal and Theodore Rolland',
    maintainer_email='rolland.theodore@student.put.poznan.pl',
    description='ROS2 package for ArUco marker detection and robot control.',
    license='Apache License 2.0',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_subscriber.camera_node:main',
        ],
    },
)
