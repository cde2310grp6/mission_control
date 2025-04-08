from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = [
        # mission control / finite state machine node
        # Node(package="mission_control", executable="mission_start", name="mission_control"),

        # amg8833 driver and casualty locator
        Node(package="casualty_location", executable="casualty_location", name="casualty_explorer"),
        Node(package="casualty_location", executable="ir_pub", name="ir_driver"),

        # frontier exploration node
        Node(package="nav2_wfd", executable="explore", name="frontier_explorer"),
    ]
    return LaunchDescription(ld)