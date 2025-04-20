#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

# Import the parser module
from ld06_lidar.ld06_parser import LD06Parser, LidarState, PACKET_LENGTH, MEASUREMENTS_PER_SCAN

class LD06LidarNode(Node):
    def __init__(self):
        super().__init__('ld06_lidar_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('scan_rate', 10.0)  # Hz
        self.declare_parameter('range_min', 0.05)  # 5cm minimum range
        self.declare_parameter('range_max', 12.0)  # 12m maximum range
        self.declare_parameter('scan_direction', 90.0) # Scan direction

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.scan_direction = self.get_parameter('scan_direction').get_parameter_value().double_value

        # Create the LIDAR parser
        self.parser = LD06Parser(debug=False)
        
        # Create publisher for laser scan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Connect to LIDAR serial port
        try:
            self.lidar_serial = serial.Serial(self.serial_port, 230400, timeout=0.5)
            self.get_logger().info(f"Connected to LIDAR on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            return

        # Initialize variables
        self.measurements = []
        self.data = b''
        self.state = LidarState.SYNC0

        # Setup timer for periodic scan publishing
        self.timer = self.create_timer(1.0/self.scan_rate, self.lidar_timer_callback)
        
        # Setup timer for transform broadcasting
        self.tf_timer = self.create_timer(0.1, self.publish_transform)  # 10Hz TF publishing

    def publish_transform(self):
        """Publish the transform from base_link to laser frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame_id
        t.child_frame_id = self.frame_id
        
        # Assuming the laser is mounted at the center of the robot
        # Adjust x, y, z as needed for your specific setup
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above base
        
        # No rotation between base_link and laser
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

    def lidar_timer_callback(self):
        """Process LIDAR data and publish scan"""
        # Main state machine
        while True:
            # Find 1st header byte
            if self.state == LidarState.SYNC0:
                self.data = b''
                self.measurements = []
                if self.lidar_serial.read() == b'\x54':
                    self.data = b'\x54'
                    self.state = LidarState.SYNC1
            
            # Find 2nd header byte
            elif self.state == LidarState.SYNC1:
                if self.lidar_serial.read() == b'\x2C':
                    self.state = LidarState.SYNC2
                    self.data += b'\x2C'
                else:
                    self.state = LidarState.SYNC0
            
            # Read remainder of the packet
            elif self.state == LidarState.SYNC2:
                self.data += self.lidar_serial.read(PACKET_LENGTH - 2)
                if len(self.data) != PACKET_LENGTH:
                    self.state = LidarState.SYNC0
                    continue
                self.measurements += self.parser.parse_packet(self.data)
                self.state = LidarState.LOCKED
            
            elif self.state == LidarState.LOCKED:
                self.data = self.lidar_serial.read(PACKET_LENGTH)
                if not self.parser.is_header_valid(self.data):
                    self.get_logger().warning("Serial sync lost")
                    self.state = LidarState.SYNC0
                    continue
                self.measurements += self.parser.parse_packet(self.data)
                if len(self.measurements) > MEASUREMENTS_PER_SCAN:
                    self.state = LidarState.PUBLISH_SCAN
            
            elif self.state == LidarState.PUBLISH_SCAN:
                # Publish the scan data
                self.publish_scan_data()
                # Get the next packet
                self.state = LidarState.LOCKED
                self.measurements = []
                break

    def publish_scan_data(self):
        """Convert LIDAR measurements to ROS LaserScan message and publish"""
        if not self.measurements:
            return
            
        # Create a LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Process the measurements
        angles_rad, distances, confidences = self.parser.process_scan_data(
            self.measurements, scan_direction=self.scan_direction)
        
        # Set LaserScan message fields
        scan_msg.angle_min = 0.0  # Start angle in radians
        scan_msg.angle_max = 2.0 * np.pi  # End angle in radians
        scan_msg.angle_increment = 2.0 * np.pi / MEASUREMENTS_PER_SCAN
        scan_msg.time_increment = 1.0 / (self.scan_rate * MEASUREMENTS_PER_SCAN)
        scan_msg.scan_time = 1.0 / self.scan_rate
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # Initialize ranges array with infinity for no returns
        ranges = [float('inf')] * MEASUREMENTS_PER_SCAN
        intensities = [0.0] * MEASUREMENTS_PER_SCAN
        
        # Filter valid ranges
        valid_indices = (distances >= self.range_min) & (distances <= self.range_max)
        
        # Populate the ranges and intensities arrays
        for i in range(len(angles_rad)):
            if not valid_indices[i]:
                continue
                
            # Normalize angle to 0-2pi
            angle_norm = angles_rad[i] % (2.0 * np.pi)
            
            # Calculate index in the output array
            index = int(angle_norm / scan_msg.angle_increment)
            if 0 <= index < MEASUREMENTS_PER_SCAN:
                # If multiple points map to the same bin, keep the closest valid one
                if ranges[index] == float('inf') or distances[i] < ranges[index]:
                    ranges[index] = distances[i]
                    intensities[index] = float(confidences[i])
        
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        
        # Publish the scan message
        self.get_logger().info(f"Scan published: {len([r for r in scan_msg.ranges if r < float('inf')])} valid points (min: {min([r for r in scan_msg.ranges if r < float('inf')] or [0.0]):.2f}m, max: {max([r for r in scan_msg.ranges if r < float('inf')] or [0.0]):.2f}m), average confidence: {sum([int(c) for c in scan_msg.intensities]) / max(len([c for c in scan_msg.intensities if c > 0]), 1):.1f}")
        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LD06LidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()