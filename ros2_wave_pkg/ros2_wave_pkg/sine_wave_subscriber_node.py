#!/usr/bin/env python3
"""
ROS2 node for subscribing to and visualizing sine wave data.

This module implements a ROS2 node that subscribes to sine wave messages,
processes the data, and provides real-time visualization using matplotlib.
The node supports dynamic parameter updates and graceful shutdown handling.
"""

from functools import partial
from threading import Event, Lock, Thread, current_thread

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from custom_interfaces.msg import SineWave
from ros2_wave_pkg.sine_wave_sub_params import sine_wave_subscriber


class WaveSubscriber(Node):
    """Subscribe to sine wave data and visualize it in real-time."""

    def __init__(self):
        super().__init__("sine_wave_subscriber")

        self.param_listener = sine_wave_subscriber.ParamListener(self)
        self.params = self.param_listener.get_params()

        # Add shutdown event flag
        self._cleanup_done = False
        self.plot_thread = None

        # Initialize data storage with thread safety
        self.data_lock = Lock()
        self.times = np.array([])
        self.values = np.array([])

        # Initialize plotting variables
        self.plot_active = False
        self.fig = None
        self.anim = None

        self.max_amplitude = None  # Track the wave amplitude

        # Start visualization if enabled
        if self.params.enable_plot:
            self.start_plot_thread()

        # Create subscription with reliable QoS
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.subscription = self.create_subscription(
            SineWave, "sine_wave", self.listener_callback, qos_profile
        )

    def start_plot_thread(self):
        """Start the plot thread."""
        self.event = Event()
        self.plot_thread = Thread(target=self.setup_plot, args=(self.event,))
        self.plot_thread.daemon = True  # Allow clean shutdown
        self.plot_thread.start()
        self.plot_active = True
        self.get_logger().info("Visualization enabled, starting plot thread")

    def listener_callback(self, msg):
        """Process incoming sine wave messages."""
        # Log data (with reduced frequency to avoid flooding)
        if (
            msg.elapsed_time % 1.0 < 0.1
        ):  # Logs every 0.1 seconds to avoid flooding during high frequency publishing
            self.get_logger().info(
                f"Received: value={msg.value:.3f}, time={msg.elapsed_time:.2f}, "
                f"amplitude={msg.amplitude:.3f}, "
                f'current_y_limits={self.ax.get_ylim() if hasattr(self, "ax") else "no axis"}'
            )
        if self.max_amplitude is None or msg.amplitude != self.max_amplitude:
            self.max_amplitude = msg.amplitude
            self.get_logger().info(
                f"Updated wave amplitude for Plots Y-axis to: {self.max_amplitude}"
            )

        # Store data if plotting is enabled
        if self.params.enable_plot:
            with self.data_lock:
                self.times = np.append(self.times, msg.elapsed_time)
                self.values = np.append(self.values, msg.value)

                # Keep buffer size in check by removing oldest data
                if len(self.times) > self.params.buffer_size:
                    self.times = self.times[-self.params.buffer_size :]
                    self.values = self.values[-self.params.buffer_size :]

    def setup_plot(self, event):
        """Set up the matplotlib plot with animation."""
        try:
            plt.style.use("seaborn-darkgrid")
            self.fig, self.ax = plt.subplots(figsize=(10, 6))

            # Initialize empty scatter plot
            self.scatter = self.ax.scatter(
                [], [], c="blue", label="Sine Wave", alpha=0.6, s=20
            )  # Size of points

            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Amplitude")
            self.ax.set_title("Sine Wave Visualization (Scatter)")
            self.ax.grid(True, alpha=0.3)
            self.ax.legend()

            # Set initial view limits
            self.ax.set_xlim(0, 10)
            self.ax.set_ylim(-5, 5)

            # Y-axis limits will be set in update_plot when we receive data
            if self.max_amplitude is not None:
                y_limit = self.max_amplitude * 1.2
                self.ax.set_ylim(-y_limit, y_limit)

            print(current_thread().name)

            # Create animation
            self.anim = FuncAnimation(
                self.fig, partial(self.update_plot, event), interval=50, blit=False
            )  # 20 FPS

            # Handle window close event
            self.fig.canvas.mpl_connect("close_event", self.on_plot_close)

            plt.show()  # This blocks until window is closed

        except Exception as e:
            self.get_logger().error(f"Error in plot setup: {str(e)}")
            self.plot_active = False

    def update_plot(self, frame, event):
        """
        Update function for matplotlib animation.

        Args:
        ----
            frame: Frame number (required by FuncAnimation but unused)
            event: Event flag to stop the animation

        """
        if self.event.is_set():
            self.anim.event_source.stop()

            plt.close(self.fig)
            self.plot_active = False
            print("plot window closed and plot_active set to False")
            return
        # if not self.plot_active:
        #     return [self.scatter]

        with self.data_lock:
            if len(self.times) < 2:
                return [self.scatter, self.ax.xaxis, self.ax.yaxis]
            times = self.times.copy()
            values = self.values.copy()

        # Update the scatter plot data
        self.scatter.set_offsets(np.c_[times, values])

        # Update x-axis - show last 10 seconds window
        current_time = times[-1]
        if current_time <= 10.0:
            # For first 10 seconds, show from 0 to 10
            self.ax.set_xlim(0, 10)
        else:
            # After 10 seconds, maintain a moving 10-second window
            self.ax.set_xlim(current_time - 10, current_time)

        # Set y-axis limits based on amplitude from subscriber
        if self.max_amplitude is not None:
            y_limit = self.max_amplitude * 1.2  # 20% padding
            self.ax.set_ylim(-y_limit, y_limit)

        # Return all artists that need to be redrawn
        return [self.scatter, self.ax.xaxis, self.ax.yaxis]

    def on_plot_close(self, event):
        """
        Handle plot window close event.

        Args:
        ----
            event: Window close event (required by matplotlib but unused)

        """
        self.plot_active = False
        self.get_logger().info("Plot window closed, stopping visualization")

    def cleanup(self):
        """Enhanced cleanup with proper thread handling."""
        if self._cleanup_done:
            return
        self._cleanup_done = True

        try:
            self.get_logger().info("Starting cleanup sequence...")

            if self.plot_active:
                self.get_logger().info("Stopping visualization...")
                self.event.set()
                self.get_logger().info("Visualization stopped")
                self.plot_thread.join()
                print("Plotting thread gracefully stopped.")

            # Clean up ROS resources
            self.destroy_subscription(self.subscription)
            self.get_logger().info("ROS resources cleaned up")

        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")


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
        node = WaveSubscriber()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
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
