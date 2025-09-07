from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap

def generate_launch_description():
    coop_perception_pkg = FindPackageShare('coop_perception_sim')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_localization = LaunchConfiguration('use_localization')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([coop_perception_pkg, 'maps', 'map.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([coop_perception_pkg, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot1', description='Robot namespace'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='true', description='Use robot namespace'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='True', description='use slam'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='False', description='use localization'
    )

    nav2_bringup_launch = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
                    launch_arguments={
                        'slam': slam,
                        'params_file': params_file,
                        'use_sim_time': use_sim_time,
                        'use_localization': use_localization,
                        'namespace': namespace,
                        'use_namespace': use_namespace,
                        'map': '',
                    }.items(),
                )
        ]
    )
    
    
    
    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_namespace_cmd,
            declare_use_namespace_cmd,
            declare_slam_cmd,
            declare_use_localization_cmd,
            nav2_bringup_launch,
        ]
    )