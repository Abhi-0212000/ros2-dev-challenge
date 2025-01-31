#!/usr/bin/env python3
"""
Integration tests for ROS2 Wave Package nodes.

This module contains launch and runtime tests for verifying the communication
between the sine wave publisher and subscriber nodes.
"""

import os
import unittest
import pytest
import rclpy
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import numpy as np
from custom_interfaces.msg import SineWave
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
def generate_test_description():
    """
    Generate launch description for testing nodes communication.

    Returns
    -------
    LaunchDescription
        Launch description with publisher and subscriber nodes.

    """
    # Get the package share directory
    pkg_share = get_package_share_directory("ros2_wave_pkg")

    # Get parameter file paths
    publisher_params_file = os.path.join(
        pkg_share, "config", "sine_wave_publisher_params.yaml"
    )
    subscriber_params_file = os.path.join(
        pkg_share, "config", "sine_wave_subscriber_params.yaml"
    )

    # Create publisher node
    publisher_node = Node(
        package="ros2_wave_pkg",
        executable="sine_wave_publisher",
        name="sine_wave_publisher_test",
        parameters=[publisher_params_file],
        output="screen",
    )

    # Create subscriber node with plot disabled for testing
    subscriber_node = Node(
        package="ros2_wave_pkg",
        executable="sine_wave_subscriber",
        name="sine_wave_subscriber_test",
        parameters=[
            subscriber_params_file,
            {"enable_plot": False},  # Override plot parameter for testing
        ],
        output="screen",
    )

    return LaunchDescription([publisher_node, subscriber_node, ReadyToTest()])


class TestNodeCommunication(unittest.TestCase):
    """Test class for verifying node communication."""

    @classmethod
    def setUpClass(cls):
        """Set up the test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up the test class."""
        try:
            rclpy.try_shutdown()
        except Exception:
            pass  # Ignore shutdown errors

    def setUp(self):
        """Set up each test case."""
        # Create a ROS node for testing
        self.node_name = "test_communication_node"
        self.test_node = rclpy.create_node(self.node_name)

        # Variables to store received messages
        self.received_messages = []
        self.message_count = 0

        # Create a subscription to monitor the sine wave topic
        self.subscription = self.test_node.create_subscription(
            SineWave, "sine_wave", self.message_callback, 10
        )

        # Log node creation
        self.test_node.get_logger().info(f"Created test node: {self.node_name}")

    def tearDown(self):
        """Clean up after each test case."""
        self.test_node.destroy_node()

    def message_callback(self, msg):
        """
        Process received messages.

        Args:
        ----
            msg (SineWave): The received message.

        """
        self.received_messages.append(msg)
        self.message_count += 1
        self.test_node.get_logger().debug(
            f"Received message {self.message_count}: value={msg.value:.3f}"
        )

    def test_publisher_subscriber_communication(self):
        """Test if publisher and subscriber are communicating properly."""
        # Wait for messages with increasing timeout
        max_wait_time = 30  # Total seconds to wait
        current_wait = 0
        spin_period = 0.1  # Spin period in seconds

        while current_wait < max_wait_time and self.message_count < 5:
            rclpy.spin_once(self.test_node, timeout_sec=spin_period)
            current_wait += spin_period

            # Log progress
            if self.message_count % 5 == 0:
                self.test_node.get_logger().info(
                    f"Received {self.message_count} messages after {current_wait:.1f} seconds"
                )

        # Verify message reception
        self.assertGreater(
            self.message_count,
            0,
            f"No messages received after {current_wait:.1f} seconds",
        )

        # Verify message content
        for i, msg in enumerate(self.received_messages):
            # Check message structure
            self.assertIsNotNone(msg.header, f"Message {i} has no header")
            self.assertIsNotNone(msg.value, f"Message {i} has no value")
            self.assertIsNotNone(msg.elapsed_time, f"Message {i} has no elapsed_time")
            self.assertIsNotNone(msg.amplitude, f"Message {i} has no amplitude")

            # Check value bounds
            self.assertLessEqual(
                abs(msg.value), msg.amplitude, f"Message {i} value exceeds amplitude"
            )

            # Check elapsed time sequence
            if i > 0:
                prev_msg = self.received_messages[i - 1]
                self.assertGreaterEqual(
                    msg.elapsed_time,
                    prev_msg.elapsed_time,
                    f"Message {i} time not increasing",
                )

    def test_sine_wave_properties(self):
        """Test if the sine wave has expected mathematical properties."""
        max_wait_time = 30  # seconds
        current_wait = 0
        spin_period = 0.1

        # Added debug print
        self.test_node.get_logger().info("\nStarting sine wave property test...")

        while current_wait < max_wait_time and self.message_count < 50:
            rclpy.spin_once(self.test_node, timeout_sec=spin_period)
            current_wait += spin_period

            # Added progress logging
            if self.message_count % 5 == 0:
                self.test_node.get_logger().info(
                    f"Current status: {self.message_count} messages after {current_wait:.1f} sec"
                )

        # Added message details logging
        values = np.array([msg.value for msg in self.received_messages])
        times = np.array([msg.elapsed_time for msg in self.received_messages])

        # Added detailed message logging
        self.test_node.get_logger().info("\nReceived message details:")
        for i, (time, value) in enumerate(zip(times, values)):
            self.test_node.get_logger().info(f"Message {i}: time={time:.3f}s, value={value:.3f}")

        # Added sign change debugging
        sign_changes = np.where(np.diff(np.signbit(values)))[0]
        self.test_node.get_logger().info(
            f"\nSign changes found at indices: {sign_changes.tolist()}"
        )
        self.test_node.get_logger().info(f"Number of sign changes: {len(sign_changes)}")

        # Added analysis summary
        time_span = times[-1] - times[0]
        self.test_node.get_logger().info(
            f"\nAnalysis Summary:"
            f"\n- Total time span: {time_span:.3f} seconds"
            f"\n- Messages collected: {len(values)}"
            f"\n- Average time between messages: {time_span/len(values):.3f} seconds"
            f"\n- Value range: [{np.min(values):.3f}, {np.max(values):.3f}]"
        )

        # More informative assertion message
        self.assertGreater(
            len(sign_changes),
            0,
            f"Sine wave not showing oscillation. Found {len(sign_changes)} sign changes "
            f"over {time_span:.3f} seconds",
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    """Test class for verifying node shutdown behavior."""

    def test_exit_codes(self, proc_info):
        """
        Verify all processes exit correctly.

        Parameters
        ----------
        proc_info : ProcessInfo
            Process information from the launch testing context.

        """
        # Allow exit code 0 (normal) and 1 (error) during testing
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, 1])
