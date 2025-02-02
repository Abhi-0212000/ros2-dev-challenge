import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ros2_wave_pkg")
    publisher_params_file = os.path.join(
        pkg_share, "config", "sine_wave_publisher_params.yaml"
    )
    subscriber_params_file = os.path.join(
        pkg_share, "config", "sine_wave_subscriber_params.yaml"
    )

    # Verify files exist
    if not os.path.exists(publisher_params_file):
        raise FileNotFoundError(
            f"Publisher params file not found: {publisher_params_file}"
        )
    if not os.path.exists(subscriber_params_file):
        raise FileNotFoundError(
            f"Subscriber params file not found: {subscriber_params_file}"
        )

    # Create the subscriber node
    subscriber_node = Node(
        package="ros2_wave_pkg",
        executable="sine_wave_subscriber",
        name="sine_wave_subscriber",
        parameters=[subscriber_params_file],
        output="screen",
    )

    # Create the publisher node
    publisher_node = Node(
        package="ros2_wave_pkg",
        executable="sine_wave_publisher",
        name="sine_wave_publisher",
        parameters=[publisher_params_file],
        output="screen",
    )

    # Wrap the publisher node in a TimerAction to delay its launch
    delayed_publisher = TimerAction(
        period=2.0, actions=[publisher_node]  # 2 second delay
    )

    return LaunchDescription(
        [
            subscriber_node,  # Launch subscriber first
            delayed_publisher,  # Launch publisher after delay
        ]
    )
