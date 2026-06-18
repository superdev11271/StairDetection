import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml

def generate_launch_description():
    """
    Generates the launch description for the staircase robot node.

    """
    
    # Get the package share directory for 'staircase_perception' pkg to find config files
    pkg_share = get_package_share_directory('staircase_perception')
    
    # Define the default path to the parameter file
    unified_estimation_config_file = os.path.join(pkg_share, 'config', 'unified_estimation_config.yaml')

    # === 1. Declare Launch Arguments ===
    declare_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value="",
        description='Namespace to prefix robot sensor data topics. Empty so the node subscribes to absolute topics (e.g. /rslidar_points, /dog_odom).'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=unified_estimation_config_file,
        description='Full path to the robot configuration file.'
    )
    
    declare_launch_marker_publisher_arg = DeclareLaunchArgument(
        'launch_marker_publisher',
        default_value='false',
        description='Whether to launch the marker publisher node.'
    )
    
    declare_simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Use simulation clock source'
    )

    declare_kill_existing_arg = DeclareLaunchArgument(
        'kill_existing',
        default_value='true',
        description='Kill any already-running staircase nodes before starting new ones (avoids stale duplicates).'
    )

    # === 2. Create Node Actions ===

    # Kill any previously-running instances so a relaunch always replaces them instead of
    # leaving stale duplicate nodes that keep publishing old detections. The '[x]' bracket in
    # each pattern is a regex that matches the real process but NOT this command line itself,
    # so the cleanup never kills itself. Runs only when 'kill_existing' is true, but always
    # exits cleanly so the nodes below start regardless.
    cleanup_existing = ExecuteProcess(
        cmd=['bash', '-c', [
            'if [ "', LaunchConfiguration('kill_existing'), '" = "true" ]; then ',
            "pkill -f '[s]taircase_estimation_robot_node'; ",
            "pkill -f '[p]ointcloud_registration_node'; ",
            "pkill -f '[s]taircase_marker_publisher'; ",
            'sleep 1; fi; true',
        ]],
        output='screen',
    )

    # Bridge: transforms the raw LiDAR cloud (e.g. /rslidar_points in 'lidar_link')
    # into the odom frame and republishes it as /registered_point_cloud, which is
    # what the estimation node expects (it does no TF lookup of its own).
    pointcloud_registration_node = Node(
        package='staircase_perception',
        executable='pointcloud_registration_node.py',
        name='pointcloud_registration_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'input_topic': '/rslidar_points',
                'output_topic': '/registered_point_cloud',
                'target_frame': 'odom',
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ]
    )

    # Node for the main staircase detector
    staircase_estimation_robot_node = Node(
        package='staircase_perception',
        executable='staircase_estimation_robot_node',
        name='staircase_estimation_robot_node', 
        output='screen',
        emulate_tty=True,
        # The 'parameters' field takes a list of file paths. Here we use the value from the launch argument.
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_topics_prefix': LaunchConfiguration('robot_namespace'),
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ]
    )

    # Conditional node for the marker publisher.
    # This node will only be launched if the 'launch_marker_publisher' argument is set to 'true'.
    stair_viz_node = Node(
        package='staircase_perception',
        executable='staircase_marker_publisher.py',
        name='stair_viz_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_topics_prefix': LaunchConfiguration('robot_namespace'),
                'use_sim_time': LaunchConfiguration('simulation'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('launch_marker_publisher'))
    )

    # === 3. Create and Return the LaunchDescription ===
    ld = LaunchDescription()

    # Add the declared arguments to the launch description
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_config_file_arg)
    ld.add_action(declare_launch_marker_publisher_arg)
    ld.add_action(declare_simulation_arg)
    ld.add_action(declare_kill_existing_arg)

    # First clean up any existing instances, then start the nodes once cleanup has finished.
    ld.add_action(cleanup_existing)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup_existing,
            on_exit=[
                pointcloud_registration_node,
                staircase_estimation_robot_node,
                stair_viz_node,
            ],
        )
    ))

    return ld

