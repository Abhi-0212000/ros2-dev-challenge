#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from custom_interfaces.srv import ProcessImage
from custom_interfaces.msg import SineWave
from std_msgs.msg import Header
import math
import cv2
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import time

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
        super().__init__('sine_wave_publisher')
        
        # Create reentrant callback group for service
        service_callback_group = ReentrantCallbackGroup()
        
        # Declare and get parameters with validation ranges
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publisher_frequency', 100.0, 
                 self._get_float_descriptor_with_min('Publishing frequency in Hz', min_value=0.0, strictly_positive=True)),
                ('amplitude', 1.0,
                 self._get_float_descriptor('Amplitude of the sine wave')),
                ('angular_frequency', 2*math.pi,  # Default 1 Hz in rad/s
                 self._get_float_descriptor_with_min('Angular frequency in rad/s', min_value=0.0)),
                ('phase', 0.0,
                 self._get_float_descriptor('Phase shift in radians'))
            ]
        )

        # Get parameters
        self.pub_freq = self.get_parameter('publisher_frequency').value
        self.amplitude = self.get_parameter('amplitude').value
        self.angular_freq = self.get_parameter('angular_frequency').value
        self.phase = self.get_parameter('phase').value

        # Validate Nyquist criterion
        self._validate_nyquist_criterion()
        
        # Create image processing service with reentrant callback group
        self.srv = self.create_service(
            ProcessImage,
            'process_image',
            self.process_image_callback,
            callback_group=service_callback_group
        )

        # Create publisher with reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10  # History depth
        )
        self.publisher = self.create_publisher(
            SineWave, 
            'sine_wave', 
            qos_profile
        )
        
        # Wait for subscriber before starting
        while self.count_subscribers('sine_wave') == 0:
            self.get_logger().info('Waiting for subscribers...')
            time.sleep(0.1)
            
        self.get_logger().info('Subscriber found, starting publication...')

        # Initialize time tracking
        self.start_time = None
        
        # Create timer for publishing
        period = 1.0 / self.pub_freq
        self.timer = self.create_timer(period, self.timer_callback)
        
        # Log parameters and initialization
        self._log_parameters()
        self.get_logger().info('Publisher and image processing service initialized')

    def _get_float_descriptor_with_min(self, description: str, min_value: float = 0.0, 
                                     strictly_positive: bool = False) -> ParameterDescriptor:
        """
        Creates a descriptor for a float parameter with a minimum value constraint.
        
        Args:
            description: Parameter description
            min_value: Minimum allowed value (inclusive)
            strictly_positive: If True, adds a note that value must be > 0
        """
        range = FloatingPointRange()
        range.from_value = min_value
        range.to_value = float('inf')
        range.step = 0.0
        
        if strictly_positive:
            description += " (must be > 0)"
        elif min_value > 0:
            description += f" (must be >= {min_value})"
            
        return ParameterDescriptor(
            description=description,
            floating_point_range=[range]
        )

    def _get_float_descriptor(self, description: str) -> ParameterDescriptor:
        """Creates a descriptor for a float parameter with no range constraints."""
        return ParameterDescriptor(description=description)

    def _validate_nyquist_criterion(self):
        """
        Validates that the publishing frequency satisfies the Nyquist criterion.
        
        For a sine wave with angular frequency ω:
        - Frequency in Hz = ω/(2π)
        - Required sampling rate = 2 * frequency
        Therefore:
        - Required publishing frequency > ω/π
        """
        min_freq = self.angular_freq / math.pi
        if self.pub_freq <= min_freq:
            raise ValueError(
                f"Publishing frequency ({self.pub_freq} Hz) is too low for "
                f"angular frequency ({self.angular_freq} rad/s). "
                f"Minimum required frequency is {min_freq:.2f} Hz "
                f"to satisfy Nyquist criterion."
            )

    def _log_parameters(self):
        """Logs the current parameter values and derived characteristics."""
        cycle_time = 2 * math.pi / self.angular_freq  # Time for one complete cycle. T = 2π/ω
        points_per_cycle = self.pub_freq * cycle_time  # Number of points per cycle. N = f*T
        freq_hz = self.angular_freq / (2 * math.pi)  # Frequency in Hz. f = ω/(2π)

        self.get_logger().info(
            f"\nSine Wave Parameters:"
            f"\n- Publishing Frequency: {self.pub_freq} Hz"
            f"\n- Amplitude: {self.amplitude}"
            f"\n- Angular Frequency: {self.angular_freq} rad/s"
            f"\n- Frequency: {freq_hz:.2f} Hz"
            f"\n- Phase: {self.phase} rad"
            f"\n\nDerived Characteristics:"
            f"\n- Time per cycle: {cycle_time:.4f} s"
            f"\n- Points per cycle: {points_per_cycle:.2f}"
        )

    def timer_callback(self):
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
            value = self.amplitude * math.sin(self.angular_freq * elapsed_time + self.phase)
            
            # Fill message with current value and parameters
            msg.value = value
            msg.elapsed_time = elapsed_time
            msg.amplitude = self.amplitude
            # msg.frequency = self.angular_freq / (2 * math.pi)  # Convert to Hz
            
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def process_image_callback(self, request, response):
        """Callback for the image processing service."""
        try:
            # Validate input path
            if not request.image_path:
                raise ValueError("Image path cannot be empty")

            # Read image
            img = cv2.imread(request.image_path)
            if img is None:
                raise FileNotFoundError(f"Could not read image at {request.image_path}")
            
            # Convert to grayscale
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Generate output path
            output_path = request.image_path.rsplit('.', 1)[0] + '_grayscale.' + request.image_path.rsplit('.', 1)[1]
            
            # Save grayscale image
            if not cv2.imwrite(output_path, gray_img):
                raise IOError(f"Failed to save image to {output_path}")
            
            self.get_logger().info(f"Image converted to grey scale and saved at: {output_path}")

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
                    ax1.axis('off')  # Hide axes
                    
                    # Display grayscale image
                    ax2.imshow(gray_img, cmap='gray')
                    ax2.set_title("Grayscale Image")
                    ax2.axis('off')  # Hide axes
                    
                    # Set the figure title
                    fig.suptitle("Image Conversion Result\nClose window to continue", fontsize=12, y=0.95)
                    
                    # Adjust subplot parameters to give specified padding
                    plt.subplots_adjust(left=0.1, right=0.9, top=0.85, bottom=0.1, wspace=0.2)
                    
                    # Block execution until window is closed
                    plt.show(block=True)
                    
                    # After window is closed, cleanup
                    plt.close('all')
                    
                except Exception as e:
                    self.get_logger().error(f'Error in visualization: {str(e)}')
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
            self.get_logger().error(f'Image processing error: {str(e)}')
            
            # Cleanup any windows in case of error
            cv2.destroyAllWindows()
        
        return response

    def cleanup(self):
        """Performs cleanup operations before node shutdown."""
        try:
            self.get_logger().info('Starting cleanup sequence...')
            
            # Cancel timer
            if hasattr(self, 'timer'):
                self.timer.cancel()
            
            # Cleanup publisher and service
            if hasattr(self, 'publisher'):
                self.destroy_publisher(self.publisher)
            if hasattr(self, 'srv'):
                self.destroy_service(self.srv)
                
            self.get_logger().info('Cleanup completed successfully')
                
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

def main(args=None):
    """Main function with enhanced error handling and cleanup."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SineWavePublisher()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
            
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()