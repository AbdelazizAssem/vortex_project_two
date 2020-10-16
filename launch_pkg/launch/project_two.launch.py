from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    talker_node = Node(
        package="gate_detection_pkg",
        executable="gate_detection",
        output='screen',
        parameters=[{"img_src" : "/home/abdelaziz/ros2_WS/src/gate_detection_pkg/frame170.jpg"}]
    )

    listener_node = Node(
        package="motion_planning",
        executable="motion_service"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld