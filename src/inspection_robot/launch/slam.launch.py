import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg         = get_package_share_directory('inspection_robot')
    urdf_file   = os.path.join(pkg, 'urdf',   'inspection_robot.urdf')
    world_file  = os.path.join(pkg, 'worlds', 'factory_world.world')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_file   = os.path.join(pkg, 'rviz',   'rviz_slam.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # ── 1. Gazebo ──────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # ── 2. Robot State Publisher ────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # ── 3. Joint State Publisher ────────────────────────────────────
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ── 4. Spawn Robot ──────────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'inspection_robot',
                    '-x', '0.0', '-y', '0.0', '-z', '0.13'
                ]
            )
        ]),

        # ── 5. Steering Animator ────────────────────────────────────────
        Node(
            package='inspection_robot',
            executable='steering_animator.py',
            name='steering_animator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ── 6. SLAM Toolbox (online async) ──────────────────────────────
        # Starts after robot is spawned and lidar is publishing.
        # Builds the map in real-time as you drive with the teleop.
        TimerAction(period=6.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': True}
                ],
            )
        ]),

        # ── 7. RViz ─────────────────────────────────────────────────────
        TimerAction(period=8.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}]
            )
        ]),

    ])
