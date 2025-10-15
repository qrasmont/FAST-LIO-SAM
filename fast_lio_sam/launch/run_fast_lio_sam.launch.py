#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as err: # parent of IOError, OSError *and* WindowsError where available
        print("Could not load YAML file. Error: ", err)
        return None 


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path', default= \
        os.path.join(get_package_share_directory("fast_lio_sam"), "config"))
    config_path_value = config_path.perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    timer_duration = LaunchConfiguration('timer_duration', default=5)
    save_map_path = LaunchConfiguration('save_map_path', default='')
    rviz_use = LaunchConfiguration('rviz', default='false')
    bag_path = LaunchConfiguration('bag_path', default='')
    buffered = LaunchConfiguration('buffered', default=True)
    map_frame = LaunchConfiguration('map_frame', default='map')
    robot_frame = LaunchConfiguration('robot_frame', default='robot')

    default_rviz_config_path = os.path.join(
        config_path_value, 'sam_rviz.rviz')

    rviz_cfg = LaunchConfiguration('rviz_cfg', default=default_rviz_config_path)
    params_file = os.path.join(
        config_path_value,
        "config_2.yaml"
    )

    params = load_yaml_file(params_file)

    fast_lio_sam_params = params["fast_lio_sam_node"]["ros__parameters"]
    fast_lio_sam_params['result']['save_map_path'] = context.perform_substitution(save_map_path)

    fast_lio_sam_node = Node(
        package="fast_lio_sam",
        executable="fast_lio_sam_node",
        name="fast_lio_sam_node",
        parameters=[fast_lio_sam_params,
                    {'basic': {'map_frame': map_frame, 'robot_frame': robot_frame}},
                    {'offline': {'bag_file': bag_path, 'buffered_read': buffered}}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    timer_action = TimerAction(
        period=timer_duration,  # Time in seconds
        actions=[fast_lio_sam_node]
    )

    return[
        timer_action,
        rviz_node
    ]

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
