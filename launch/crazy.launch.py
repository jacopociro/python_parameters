from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='crazy_node',
            name='crazy_node',
            #namespace='cf_0',
            #output='screen',
            parameters=['/home/gonazza/DroneMission_ws/src/python_parameters/config/crazyswarmconfig.yaml',
                        {'drone_name': "cf_0"},
                        {'drone_id': 0},
                        {'pos_init': [0.0, 0.0, 0.0]},
                        {'yaw_init': 0.0}]

        ),

        # Node(
        #     package='python_parameters',
        #     executable='crazy_node',
        #     name='crazy_node',
        #     #namespace='cf_1',
        #     #output='screen',
        #     parameters=['/home/gonazza/DroneMission_ws/src/python_parameters/config/crazyswarmconfig.yaml',
        #                 {'drone_name': "cf_1"},
        #                 {'drone_id': 1},
        #                 {'pos_init': [1.0, 0.0, 0.0]},
        #                 {'yaw_init': 0.0}]
        # ),
        Node(
            package='python_parameters',
            executable='leader',
            name='leader',
            #namespace='cf_1',
            output='screen',
            parameters=['/home/gonazza/DroneMission_ws/src/python_parameters/config/crazyswarmconfig.yaml'],

        ),

        # Node(
        #     package='python_parameters',
        #     executable='obstacles',
        #     name='obstacles',
        #     #namespace='cf_1',
        #     output='screen',
            
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/gonazza/DroneMission_ws/src/python_parameters/config/crazy.config.rviz'], # Argomento per il file .rviz
            output='screen')

    ])

#   ros__parameters:
#     drones:
#       drone_0:
#         drone_name: "cf_0"
#         drone_id: 0.0
#         x_init: 0.0
#         y_init: 0.0
#         z_init: 0.0
#         yaw_init: 0.0
#       drone_1:
#         drone_name: "cf_1"
#         drone_id: 1.0
#         x_init: 1.0
#         y_init: 0.0
#         z_init: 0.0
#         yaw_init: 0.0