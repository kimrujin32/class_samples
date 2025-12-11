from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'cowork_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'two_turtlebot3s = cowork_robot.two_turtlebot3s:main',
            'spawn_one = cowork_robot.spawn_one:main',    
            'publish_topic = cowork_robot.publish_topic:main',                     
        ],
    },
)
