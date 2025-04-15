from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        Node(
            package="mission_control",
            executable="missionStart",
            name="mission_control",
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package="casualty_location",
            executable="casualty_location",
            name="casualty_explorer",
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package="casualty_location",
            executable="casualty_saver",
            name="casualty_saver",
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package="nav2_wfd",
            executable="explore",
            name="frontier_explorer",
        ),

        Node(
            package="aligner_node",
            executable="aligner",
            name="aligner_node",
        )
    ]

    return LaunchDescription(ld)
