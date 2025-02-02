#!/usr/bin/env python3
"""
ROS2 node for publishing sine wave data and providing image processing services.

This module implements a ROS2 node that generates and publishes sine wave data
and provides a service for converting color images to grayscale.
"""

import math
import os

import cv2
import matplotlib.pyplot as plt
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from custom_interfaces.msg import SineWave
from custom_interfaces.srv import ProcessImage
from ros2_wave_pkg.sine_wave_pub_params import sine_wave_publisher


class SineWavePublisher(Node):
    """
    ROS2 Node that publishes sine wave data and provides image processing service.

    The sine wave is generated using the formula:
        y(t) = A * sin(ωt + φ)
    where:
        A = Amplitude
        ω = Angular frequency (rad/s)
        φ = Phase shift (rad)
        t = Time (s)
    """

    def __init__(self):
        super().__init__("sine_wave_publisher")

        self.param_listener = sine_wave_publisher.ParamListener(self)
        self.params = self.param_listener.get_params()

        # Create reentrant callback group for service
        service_callback_group = ReentrantCallbackGroup()

        # Create image processing service with reentrant callback group
        self.srv = self.create_service(
            ProcessImage,
            "process_image",
            self.process_image_callback,
            callback_group=service_callback_group,
        )

        # Create publisher with reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10  # History depth
        )
        self.publisher = self.create_publisher(SineWave, "sine_wave", qos_profile)

        # Initialize time tracking
        self.start_time = None

        # Create timer for publishing
        period = 1.0 / self.params.publisher_frequency
        self.publisher_timer = self.create_timer(period, self.publisher_timer_callback)

        self.param_listener_timer = self.create_timer(1, self.param_listener_callback)

        # Log parameters and initialization
        self._log_parameters()
        self.get_logger().info("Publisher and image processing service initialized.")

    def param_listener_callback(self):
        """
        Timer callback function to handle parameter updates.

        This method checks if any parameters have changed since the last update.
        If changes are detected, it refreshes the dynamic parameters and updates
        the stored parameter values.
        """
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

    def _log_parameters(self):
        """Log the current parameter values and derived characteristics."""
        cycle_time = (
            2 * math.pi / self.params.angular_frequency
        )  # Time for one complete cycle. T = 2π/ω
        points_per_cycle = (
            self.params.publisher_frequency * cycle_time
        )  # Number of points per cycle. N = f*T
        freq_hz = self.params.angular_frequency / (
            2 * math.pi
        )  # Frequency in Hz. f = ω/(2π)

        self.get_logger().info(
            f"\nSine Wave Parameters:"
            f"\n- Publishing Frequency: {self.params.publisher_frequency} Hz"
            f"\n- Amplitude: {self.params.amplitude}"
            f"\n- Angular Frequency: {self.params.angular_frequency} rad/s"
            f"\n- Frequency: {freq_hz:.2f} Hz"
            f"\n- Phase: {self.params.phase} rad"
            f"\n\nDerived Characteristics:"
            f"\n- Time per cycle: {cycle_time:.4f} s"
            f"\n- Points per cycle: {points_per_cycle:.2f}"
        )

    def publisher_timer_callback(self):
        """
        Timer callback function that computes and publishes the sine wave value.

        The sine wave is computed as:
            y(t) = A * sin(ωt + φ)
        where t is the elapsed time since the start of publishing.
        """
        try:
            msg = SineWave()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Initialize start_time on first callback
            current_time = self.get_clock().now()
            if self.start_time is None:
                self.start_time = current_time
                elapsed_time = 0.0
            else:
                elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

            # Calculate sine wave value
            value = self.params.amplitude * math.sin(
                self.params.angular_frequency * elapsed_time + self.params.phase
            )

            # Fill message with current value and parameters
            msg.value = value
            msg.elapsed_time = elapsed_time
            msg.amplitude = self.params.amplitude
            # msg.frequency = self.params.angular_frequency / (2 * math.pi)  # Convert to Hz

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

    @staticmethod
    def get_test_image_path():
        """Get the absolute path to the test image."""
        package_share_dir = get_package_share_directory("ros2_wave_pkg")
        test_image_path = os.path.join(
            package_share_dir, "test_images", "test_image.jpg"
        )
        return test_image_path

    def process_image_callback(self, request, response):
        """
        Process images and convert them to grayscale.

        Handle the image processing service request by converting the input image
        to grayscale and optionally displaying results.

        Args
        ----
        request: Service request containing image path and display flag
        response: Service response for returning results

        Returns
        -------
        ProcessImage.Response: Service response with conversion results

        """
        try:
            if request.image_path == "test":
                request.image_path = self.get_test_image_path()
                self.get_logger().info(f"Using test image at: {request.image_path}")

            # Validate input path
            if not request.image_path:
                raise ValueError("Image path cannot be empty")

            # Read image
            img = cv2.imread(
                request.image_path
            )  # pylint: disable=c-extension-no-member
            if img is None:
                raise FileNotFoundError(f"Could not read image at {request.image_path}")

            # Convert to grayscale
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Generate output path
            output_path = (
                request.image_path.rsplit(".", 1)[0]
                + "_grayscale."
                + request.image_path.rsplit(".", 1)[1]
            )

            # Save grayscale image
            if not cv2.imwrite(output_path, gray_img):
                raise IOError(f"Failed to save image to {output_path}")

            self.get_logger().info(
                f"Image converted to grey scale and saved at: {output_path}"
            )

            # Handle visualization if requested
            if request.show_visualization:
                try:
                    # Create figure with subplots
                    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

                    # Display original image
                    # Convert BGR to RGB for correct color display
                    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    ax1.imshow(rgb_img)
                    ax1.set_title("Original Image")
                    ax1.axis("off")  # Hide axes

                    # Display grayscale image
                    ax2.imshow(gray_img, cmap="gray")
                    ax2.set_title("Grayscale Image")
                    ax2.axis("off")  # Hide axes

                    # Set the figure title
                    fig.suptitle(
                        "Image Conversion Result\nClose window to continue",
                        fontsize=12,
                        y=0.95,
                    )

                    # Adjust subplot parameters to give specified padding
                    plt.subplots_adjust(
                        left=0.1, right=0.9, top=0.85, bottom=0.1, wspace=0.2
                    )

                    # Block execution until window is closed
                    plt.show(block=True)

                    # After window is closed, cleanup
                    plt.close("all")

                except Exception as e:
                    self.get_logger().error(f"Error in visualization: {str(e)}")
                    # Continue with service response even if visualization fails

            response.success = True
            response.message = (
                f"Image successfully converted to grayscale and saved at: {output_path}"
                f"{' (visualization shown)' if request.show_visualization else ''}"
            )
            response.processed_image_path = output_path
            self.get_logger().info("Response sent successfully")

        except Exception as e:
            response.success = False
            response.message = f"Error processing image: {str(e)}"
            response.processed_image_path = ""
            self.get_logger().error(f"Image processing error: {str(e)}")

        return response

    def cleanup(self):
        """Clean up node resources before shutdown."""
        try:
            self.get_logger().info("Starting cleanup sequence...")

            # Cancel timer
            if hasattr(self, "publisher_timer"):
                self.publisher_timer.cancel()

            if hasattr(self, "param_listener_timer"):
                self.param_listener_timer.cancel()

            # Cleanup publisher and service
            if hasattr(self, "publisher"):
                self.destroy_publisher(self.publisher)

            if hasattr(self, "srv"):
                self.destroy_service(self.srv)

        except Exception as e:
            # self.get_logger().error(f"Error during cleanup: {str(e)}")
            print(f"Error during cleanup: {str(e)}")


def main(args=None):
    """
    Run the sine wave publisher node.

    Args:
    ----
    args: Command-line arguments passed to rclpy.init

    """
    rclpy.init(args=args)
    node = None

    try:
        node = SineWavePublisher()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            # First cancel the timers in the node
            if node is not None:
                node.cleanup()
                print("Node cleanup completed successfully")
            # Then shutdown the executor
            if executor is not None:
                executor.shutdown()
                print("Executor shutdown completed successfully")

    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        # Cleanup in reverse order of creation
        if executor is not None:
            executor.shutdown()
            print("Executor shutdown completed in finally block")
        if node is not None:
            node.destroy_node()
            print("Node destroyed in finally block")
        if rclpy.ok():
            rclpy.shutdown()
            print("ROS2 shutdown completed in finally block")


if __name__ == "__main__":
    main()
