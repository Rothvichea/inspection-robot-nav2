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
    rviz_file   = os.path.join(pkg, 'rviz',   'rviz.rviz')
    map_file    = os.path.join(pkg, 'maps',   'factory_map.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    ekf_params  = os.path.join(pkg, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # ── 1. Gazebo ──────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # ── 2. Robot State Publisher ───────
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

        # ── 3. EKF (robot_localization) ────
        # Fuses /odom + /imu → /odometry/filtered
        # publish_tf=true: publishes odom→base_footprint TF
        # planar_move publish_odom_tf is disabled so only EKF publishes this TF
        TimerAction(period=4.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params, {'use_sim_time': True}],
                remappings=[('odometry/filtered', '/odom/filtered')]
            )
        ]),

        # ── 5. Spawn Robot ─────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', 'inspection_robot',
                    '-topic',  'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.2'
                ]
            )
        ]),

        # ── 6. Map Server ──────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_file
                }]
            )
        ]),

        # ── 7. AMCL ────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params]
            )
        ]),

        # ── 8. Lifecycle Manager localization
        TimerAction(period=7.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            )
        ]),

        # ── 9. Nav2 Navigation stack ───────
        # Delay to 14 s — gives localization (t=7 s) time to fully activate
        # before the navigation lifecycle manager starts.
        TimerAction(period=14.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file':  nav2_params,
                    'map_subscribe_transient_local': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]),

        # ── 10. Auto Initial Pose ──────────
        # Starts at t=28 s — Nav2 lifecycle manager activates from t=14 s
        # and needs ~10 s to finish; initial pose must arrive after AMCL is active.
        TimerAction(period=28.0, actions=[
            Node(
                package='inspection_robot',
                executable='initial_pose_publisher.py',
                name='initial_pose_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]),

        # ── 11. Steering Animator ──────────
        # Converts cmd_vel → steering joint angles so wheels visually pivot
        Node(
            package='inspection_robot',
            executable='steering_animator.py',
            name='steering_animator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ── 12. RViz ───────────────────────
# ── 12. Foxglove Bridge ───────────
        TimerAction(period=30.0, actions=[
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