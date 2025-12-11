from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'turtlebot3_gui'

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
    maintainer='love',
    maintainer_email='love@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_point = turtlebot3_gui.turtlebot3_point:main',
            'turtlebot3_move = turtlebot3_gui.turtlebot3_move:main',
            'send_goal = turtlebot3_gui.send_goal:main',
            'pub_topic = turtlebot3_gui.pub_topic:main',
            'sub_topic = turtlebot3_gui.sub_topic:main',

            'topic_pub = turtlebot3_gui.topic_pub:main',
            'topic_sub = turtlebot3_gui.topic_sub:main',       
            'topic_sub_qt = turtlebot3_gui.topic_sub_qt:main',  
            'service_server = turtlebot3_gui.service_server:main',
            'service_client = turtlebot3_gui.service_client:main',
            'nav_server = turtlebot3_gui.nav_server:main',
            'nav_client = turtlebot3_gui.nav_client:main',
            'send_goal_wait = turtlebot3_gui.send_goal_wait:main',         
            'nav_goal = turtlebot3_gui.nav_goal:main',       
            'delivery = turtlebot3_gui.delivery:main',         
            'action_server = turtlebot3_gui.action_server:main',   
            'action_client = turtlebot3_gui.action_client:main',      
            'turtlebot3_gui3 = turtlebot3_gui.turtlebot3_gui3:main',       
            'menu_service = turtlebot3_gui.menu_service:main',  
            'menu_server = turtlebot3_gui.menu_server:main',      
            'initpose_gui = turtlebot3_gui.initpose_gui:main',      
            'nav2goal_gui = turtlebot3_gui.nav2goal_gui:main',          
            'sqlite_gui = turtlebot3_gui.sqlite_gui:main',                                                                
        ],
    },
)
