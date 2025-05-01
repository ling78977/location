import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    pkg_dir = get_package_share_directory("bring_up")
    default_config_path = os.path.join(pkg_dir, "config", "node_param.yaml")

    map_pub_node = Node(
        package="my_nav",
        executable="map_publisher_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[default_config_path],
    )
    global_planner_node = Node(
        package="my_nav",
        executable="global_planner_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[default_config_path],
    )
    robot_control_node = Node(
        package="my_nav",
        executable="robot_control_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[default_config_path],
    )
    rm_serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[default_config_path],
    )
    fast_location_node = Node(
        package="fast_lio",
        executable="fastlio_location",
        parameters=[default_config_path, {"use_sim_time": False}],
        output="screen",
    )
    other_launch_file_path = PathJoinSubstitution([get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py'])
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file_path),
    )
    foxglove_cmd=ExecuteProcess(cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml', 'send_buffer_limit:=1000000000'],output='screen')

    ld = LaunchDescription()
    ld.add_action(map_pub_node)
    ld.add_action(global_planner_node)
    ld.add_action(robot_control_node)
    # ld.add_action(rm_serial_driver_node)
    ld.add_action(fast_location_node)
    ld.add_action(include_other_launch)
    ld.add_action(foxglove_cmd)


    return ld
