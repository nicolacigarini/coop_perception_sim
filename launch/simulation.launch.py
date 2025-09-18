from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument("namespace_robot_1", default_value='robot1', description="Robot #1 namespace"),
    DeclareLaunchArgument("namespace_robot_2", default_value='robot2', description="Robot #2 namespace"),
    DeclareLaunchArgument("namespace_robot_3", default_value="robot3", description="Robot #3 namespace"),
    DeclareLaunchArgument("world", default_value="sparcs", description="Simulation world")
]


def launch_setup(context:LaunchContext, *args, **kwargs):
    robot1_namespace = LaunchConfiguration("namespace_robot_1")
    robot2_namespace = LaunchConfiguration("namespace_robot_2")
    robot3_namespace = LaunchConfiguration("namespace_robot_3")
    world = LaunchConfiguration('world')

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gz_sim_bringup_sparcs"),
                'launch',
                'gz.launch.py'
            ])
        ),
        launch_arguments=[
            ('world', world)
        ]
    )

    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gz_sim_bringup_sparcs"),
                'launch',
                'UGV_agent_spawn.launch.py'
            ])
        ),
        launch_arguments=[
            ('namespace', robot1_namespace),
            ('UGV_model', 'turtlebot'),
            ('x', '0.0'),
            ('y', '0.0'),
            ('Y','0.0')
        ],
    )

    robot2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gz_sim_bringup_sparcs"),
                'launch',
                'UGV_agent_spawn.launch.py'
            ])
        ),
        launch_arguments=[
            ('namespace', robot2_namespace),
            ('UGV_model', 'turtlebot'),
            ('x', '1.5'),
            ('y', '0.5'),
            ('Y','3.14')
        ],
    )
    multi_lidar_slam_params = PathJoinSubstitution([FindPackageShare("coop_perception_sim"), "config", "multiLidarSlam.yaml"])
    multi_lidar_slam = Node(
        package="coop_perception_sim",
        executable="multi_lidar_slam",
        output="screen",
        parameters=[{'use_sim_time': True}, multi_lidar_slam_params],
    )

    odom_map_remapper_params = PathJoinSubstitution([FindPackageShare("coop_perception_sim"), "config", "odomMapRemapper.yaml"])
    robot1_frame_repub =  Node(
        package='coop_perception_sim', 
        executable='odom_map_remapper',
        output='screen',        
        namespace= robot1_namespace,
        parameters=[{'use_sim_time': True}, odom_map_remapper_params],
    )

    robot2_frame_repub =  Node(
        package='coop_perception_sim', 
        executable='odom_map_remapper',
        output='screen',        
        namespace= robot2_namespace,
        parameters=[{'use_sim_time': True}, odom_map_remapper_params],
    )


    return [gz_launch, robot1_spawn, robot1_frame_repub, robot2_spawn, robot2_frame_repub,multi_lidar_slam]


def generate_launch_description():
    return LaunchDescription(
        ARGUMENTS + 
        [OpaqueFunction(function=launch_setup)]
    )