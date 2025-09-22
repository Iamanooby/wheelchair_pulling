# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# MAP_NAME='aws_warehouse' #change to the name of your own map here\
# MAP_NAME='b2l4_big_3'
# MAP_NAME='b2l4_big_3_lined'
# MAP_NAME='b2l4_small_2'
# MAP_NAME='map_dyson'
MAP_NAME='dsis3_map'
# MAP_NAME='canteen_map2'
# MAP_NAME='playground'
# MAP_NAME= 'b2l4_big_3_comp'
# MAP_NAME= 'b2l4_big_3_comp_crop'


def generate_launch_description():
    # depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('wheelchair_navigation'), 'rviz', 'tb3.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('wheelchair_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('wheelchair_navigation'), 'config', 'navigation.yaml']
    )

    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'), 'launch', 'rplidar_a3_launch.py']
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='sim', 
        #     default_value='true',
        #     description='Enable use_sime_time to true'
        # ),

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        # DeclareLaunchArgument(
        #     name='rviz', 
        #     default_value='false',
        #     description='Run rviz'
        # ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={
                'frame_id': 'base_scan'
            }.items()
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_path],
        #     condition=IfCondition(LaunchConfiguration("rviz")),
        #     parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '-0.30', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
        # ),

        Node(
            package='convert_cmd_vel',
            executable='output_holo_tow',
            name='output_holo_tow',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='convert_cmd_vel',
            executable='filter_scan',
            name='filter_scan',
            output='screen',
            emulate_tty=True,
        ),

    ])