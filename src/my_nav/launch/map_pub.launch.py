import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('my_nav')
    default_config_path = os.path.join(package_path, 'config',"map_pub_params.yaml")


    
    map_pub_node = Node(
        package='my_nav',
        executable='map_publisher_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[default_config_path],
    )
    global_planner_node = Node(
        package='my_nav',
        executable='map_publisher_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[default_config_path],
    )
    

    ld = LaunchDescription()
    ld.add_action(map_pub_node)
    

    return ld



