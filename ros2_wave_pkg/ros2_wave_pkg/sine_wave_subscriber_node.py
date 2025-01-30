#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import SineWave
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from threading import Lock, Thread
from ros2_wave_pkg.sine_wave_sub_params import sine_wave_subscriber

class WaveSubscriber(Node):
    """
    ROS2 Node that subscribes to sine wave data and optionally visualizes it.
    """

    def __init__(self):
        super().__init__('sine_wave_subscriber')
        
        self.param_listener = sine_wave_subscriber.ParamListener(self)
        self.params = self.param_listener.get_params()
        
        # Add shutdown event flag
        self._cleanup_done = False
        self._shutdown_event = False
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
            self.plot_thread = Thread(target=self.setup_plot)
            self.plot_thread.daemon = True  # Make thread daemon so it doesn't block shutdown
            self.plot_thread.start()
            self.plot_active = True
            self.get_logger().info('Visualization enabled, starting plot thread')
        else:
            self.get_logger().info('Visualization disabled, running in logging-only mode')
            
        # Create subscription with reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.subscription = self.create_subscription(
            SineWave,
            'sine_wave',
            self.listener_callback,
            qos_profile
        )


    def listener_callback(self, msg):
        """Callback for receiving sine wave data."""
        # Log data (with reduced frequency to avoid flooding)
        if msg.elapsed_time % 1.0 < 0.1:  # Logs every 0.1 seconds to avoid flooding during high frequency publishing
            self.get_logger().info(
                f'Received: value={msg.value:.3f}, time={msg.elapsed_time:.2f}, '
                f'amplitude={msg.amplitude:.3f}, '
                f'current_y_limits={self.ax.get_ylim() if hasattr(self, "ax") else "no axis"}'
            )
        if self.max_amplitude is None or msg.amplitude != self.max_amplitude:
            self.max_amplitude = msg.amplitude
            self.get_logger().info(f'Updated wave amplitude for Plots Y-axis to: {self.max_amplitude}')

        # Store data if plotting is enabled
        if self.params.enable_plot:
            with self.data_lock:
                self.times = np.append(self.times, msg.elapsed_time)
                self.values = np.append(self.values, msg.value)
                
                # Keep buffer size in check by removing oldest data
                if len(self.times) > self.params.buffer_size:
                    self.times = self.times[-self.params.buffer_size:]
                    self.values = self.values[-self.params.buffer_size:]

    def setup_plot(self):
        """Sets up the matplotlib plot with animation."""
        try:
            plt.style.use('seaborn-darkgrid')
            self.fig, self.ax = plt.subplots(figsize=(10, 6))
            self.line, = self.ax.plot([], [], 'b-', label='Sine Wave', linewidth=2)
            
            self.ax.set_xlabel('Time (s)')
            self.ax.set_ylabel('Amplitude')
            self.ax.set_title('Sine Wave Visualization')
            self.ax.grid(True, alpha=0.3)
            self.ax.legend()
            
            # Set initial view limits
            self.ax.set_xlim(0, 10)
            # self.ax.set_ylim(-2, 2)
            # Y-axis limits will be set in update_plot when we receive data
            if self.max_amplitude is not None:
                y_limit = self.max_amplitude * 1.2
                self.ax.set_ylim(-y_limit, y_limit)
            
            # Create animation
            self.anim = FuncAnimation(
                self.fig, 
                self.update_plot,
                interval=50,  # 20 FPS
                blit=False
            )
            
            # Handle window close event
            self.fig.canvas.mpl_connect('close_event', self.on_plot_close)
            
            plt.show()  # This blocks until window is closed
            
        except Exception as e:
            self.get_logger().error(f'Error in plot setup: {str(e)}')
            self.plot_active = False
    
    def update_plot(self, frame):
        """Update function for matplotlib animation."""
        if not self.plot_active:
            return [self.line]

        with self.data_lock:
            if len(self.times) < 2:  
                return [self.line, self.ax.xaxis, self.ax.yaxis]
            times = self.times.copy()
            values = self.values.copy()

        # Update the line data
        self.line.set_data(times, values)
        
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
        return [self.line, self.ax.xaxis, self.ax.yaxis]

    def on_plot_close(self, event):
        """Handle plot window close event."""
        self.plot_active = False
        self.get_logger().info('Plot window closed, stopping visualization')

    def cleanup(self):
        """Enhanced cleanup with proper thread handling."""
        if self._cleanup_done:
            return
        self._cleanup_done = True
        
        try:
            self.get_logger().info('Starting cleanup sequence...')
            
            # First, mark for shutdown and stop plotting
            self._shutdown_event = True
            self.plot_active = False
            
            if self.params.enable_plot:
                # Stop animation if it exists
                if hasattr(self, 'anim') and self.anim is not None:
                    self.get_logger().info('Stopping animation...')
                    try:
                        self.anim.event_source.stop()
                    except Exception as e:
                        self.get_logger().error(f'Error stopping animation: {str(e)}')
                    self.anim = None
                
                # Close plot windows
                plt.close('all')
                
                # Wait for plotting thread
                if self.plot_thread is not None:
                    if self.plot_thread.is_alive():
                        self.get_logger().info('Waiting for plot thread to terminate...')
                        self.plot_thread.join(timeout=1.0)
                        if self.plot_thread.is_alive():
                            self.get_logger().warn('Plot thread still alive after timeout')
                    else:
                        self.get_logger().info('Plot thread already terminated')
                
                self.get_logger().info('Visualization cleanup completed')

            # Clean up ROS resources
            self.destroy_subscription(self.subscription)
            self.get_logger().info('ROS resources cleaned up')

        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

def main(args=None):
    """Main function with better shutdown handling."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = WaveSubscriber()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        
        def signal_handler(sig, frame):
            """Handle shutdown signals."""
            nonlocal node
            if node is not None and not node._cleanup_done:
                node.get_logger().info('Shutdown signal received')
                try:
                    # Stop executor first
                    executor.shutdown()
                    # Then cleanup node
                    node.cleanup()
                    node.destroy_node()
                    # Finally shutdown rclpy
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception as e:
                    print(f'Error during shutdown: {e}')
        
        # Setup signal handlers
        import signal
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            signal_handler(signal.SIGINT, None)  # Handle keyboard interrupt same as signals
            
    except Exception as e:
        print(f'Error in main: {str(e)}')
        if node is not None and not node._cleanup_done:
            node.cleanup()

if __name__ == '__main__':
    main()