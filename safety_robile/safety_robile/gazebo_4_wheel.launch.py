#!/usr/bin/env python3
# Authors: Deebul Nair

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch.actions import TimerAction


def generate_launch_description():
    # lc = LaunchContext()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-3.5')

    x_pose2 = LaunchConfiguration('x_pose2', default='-2.0')
    y_pose2 = LaunchConfiguration('y_pose2', default='-6.0')

    x_pose3 = LaunchConfiguration('x_pose3', default='-2.0')
    y_pose3 = LaunchConfiguration('y_pose3', default='-8.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "gazebo",
                    "gazebo_robile_laserscanner_camera.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
    )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    spawn_robot_gazebo_cmd = Node(package='gazebo_ros',
                                  executable='spawn_entity.py',
                                  arguments=['-topic', 'robot_description',
                                             '-entity', 'robile',
                                             '-x', x_pose,
                                             '-y', y_pose,
                                             ],
                                  output='screen')

    spawn_robot_gazebo_cmd2 = TimerAction(
    period=5.0,
    actions = [Node(package='gazebo_ros',
                                      executable='spawn_entity.py',
                                      arguments=['-topic', 'robot_description',
                                                 '-entity', 'robile2',
                                                 '-x', x_pose2,
                                                 '-y', y_pose2,
                                                 '-robot_namespace', '/robile2'],
                                      output='screen')]

    )


    spawn_robot_gazebo_cmd3 = TimerAction(
    period=8.0, 
    actions = [Node(package='gazebo_ros',
                                      executable='spawn_entity.py',
                                      arguments=['-topic', 'robot_description',
                                                 '-entity', 'robile3',
                                                 '-x', x_pose3,
                                                 '-y', y_pose3,
                                                 '-robot_namespace', '/robile3'],
                                      output='screen')]
    )




    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    )

    static_transform_cmd = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_footprint", "base_link"]
                                )

    nodes = [
        # rviz_cmd,

        gzserver_cmd,
        gzclient_cmd,
        robot_state_pub_cmd,
        spawn_robot_gazebo_cmd,
        spawn_robot_gazebo_cmd2,
        spawn_robot_gazebo_cmd3,
        static_transform_cmd,
        joint_state_publisher_cmd
    ]

    return LaunchDescription(nodes)

    # urdf_path = os.path.join(
    #    get_package_share_directory('robile_description'),
    #    'robots',
    #    urdf_file_name)

    # with open(urdf_path, 'r') as infp:
    #    robot_desc = infp.read()

    # return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),

    #    Node(
    #        package='robot_state_publisher',
    #        executable='robot_state_publisher',
    #        name='robot_state_publisher',
    #        output='screen',
    #        parameters=[{
    #            'use_sim_time': use_sim_time,
    #            'robot_description': robot_desc
    #        }],
    #    ),
    # ])
