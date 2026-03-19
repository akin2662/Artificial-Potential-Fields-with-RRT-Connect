"""
navigation.launch.py
---------------------
Launches the full navigation stack:
  1. Python planner service node  (must start BEFORE Nav2)
  2. Gazebo simulation
  3. Nav2 localization
  4. Nav2 navigation (planner, controller, etc.)
  5. RViz

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    pkg_share    = get_package_share_directory('apf_rrtc_plugin')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')

    params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    map_file    = '/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml'
    model_path = os.path.join(
        get_package_share_directory('apf_rrtc_plugin'), 'models')
    set_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=model_path + ':/opt/ros/humble/share/turtlebot3_gazebo/models'
)
    

    # ── Launch arguments ──────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true')

    # ── 1. Python planner service node ────────────────────────────────────
    # This MUST start before Nav2 because the C++ plugin waits for
    # the /compute_path service in its configure() method.
    planner_service_node = Node(
        package='apf_rrtc_plugin',
        executable='planner_service_node.py',
        name='apf_rrtc_planner_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── 2. Gazebo ─────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'extra_gazebo_args': '--ros-args -p use_sim_time:=true'
        }.items())

    # ── 3. Nav2 localization ──────────────────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map':          map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  params_file,
        }.items())

    # ── 4. Nav2 navigation — delayed 5s to let localization start first ───
    navigation = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file':  params_file,
            }.items())])

    # ── 5. RViz ───────────────────────────────────────────────────────────
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')}])

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        planner_service_node,
        set_model_path,   # Python service node — starts first
        gazebo,
        localization,
        navigation,             # delayed 5s after localization
        rviz,
    ])
