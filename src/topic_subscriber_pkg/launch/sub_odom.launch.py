from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
        package="topic_subscriber_pkg",
        executable="odom_subscriber_node",
        output="screen")
    ])