import glob
import os
from setuptools import find_packages, setup

package_name = 'bigbot_gazebo'

# Helper function to include all files in the models directory while preserving structure
def get_model_files():
    model_files = []
    for root, dirs, files in os.walk('models'):
        if files:
            # Get relative path from models directory
            rel_path = os.path.relpath(root, 'models')
            target_dir = os.path.join('share', package_name, 'models', rel_path)
            file_paths = [os.path.join(root, f) for f in files]
            model_files.append((target_dir, file_paths))
    return model_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'params'), glob.glob('params/*')),
        (os.path.join('share', package_name, 'worlds'), glob.glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        ]
        + get_model_files() ,
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='Edward Hage',
    maintainer_email='edward@confirmatrobotics.com',
    description='Bigbot simulation with GPS and Lidar in Gazebo',
    license='Apache-2.0',
    entry_points={
       'console_scripts': [
            'odom_connect = bigbot_gazebo.odom_connect:main',
            'drive_status_simulator = bigbot_gazebo.drive_status_simulator:main',
        ],
    },
)

