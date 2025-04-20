#!/usr/bin/env python3

import numpy as np
import struct
from enum import Enum

# ----------------------------------------------------------------------
# System Constants
# ----------------------------------------------------------------------
# At the default rotation speed (~3600 deg/s) the system outputs about 
# 480 measurements in a full rotation.
MEASUREMENTS_PER_SCAN = 480

# ----------------------------------------------------------------------
# Main Packet Format
# ----------------------------------------------------------------------
# All fields are little endian
# Header (1 byte) = 0x54
# Length (1 byte) = 0x2C (assumed to be constant)
# Speed (2 bytes) Rotation speed in degrees per second
# Start angle (2 bytes) divide by 100.0 to get angle in degrees
# Data Measurements (MEASUREMENT_LENGTH * 3 bytes)
#                   See "Format of each data measurement" below
# Stop angle (2 bytes) divide by 100.0 to get angle in degrees
# Timestamp (2 bytes) In milliseconds
# CRC (1 bytes) Poly: 0x4D, Initial Value: 0x00, Final Xor Value: 0x00
#               Input reflected: False, Result Reflected: False
#               http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
# Format of each data measurement
# Distance (2 bytes) # In millimeters
# Confidence (1 byte)

# ----------------------------------------------------------------------
# Packet format constants
# ----------------------------------------------------------------------
PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12 
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

class LidarState(Enum):
    """Enum for the state machine states"""
    SYNC0 = 0
    SYNC1 = 1
    SYNC2 = 2
    LOCKED = 3
    PUBLISH_SCAN = 4

class LD06Parser:
    """Parser for the LD06 LIDAR data packets"""
    
    def __init__(self, debug=False):
        """Initialize the LD06 parser
        
        Args:
            debug (bool): Enable debug printing
        """
        self.debug = debug

    def parse_packet(self, data):
        """Parse binary LIDAR packet data into angle, distance, and confidence values
        
        Args:
            data (bytes): Raw binary packet data
            
        Returns:
            list: List of tuples (angle, distance, confidence)
        """
        if len(data) != PACKET_LENGTH:
            return []
            
        try:
            # Extract data
            length, speed, start_angle, *pos_data, stop_angle, timestamp, crc = \
                struct.unpack(MESSAGE_FORMAT, data)
            
            # Scale values
            start_angle = float(start_angle) / 100.0
            stop_angle = float(stop_angle) / 100.0
            
            # Unwrap angle if needed and calculate angle step size
            if stop_angle < start_angle:
                stop_angle += 360.0
            step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1)
            
            # Get the angle for each measurement in packet
            angle = [start_angle + step_size * i for i in range(0, MEASUREMENT_LENGTH)]
            distance = pos_data[0::2]  # in millimeters
            confidence = pos_data[1::2]
            
            if self.debug:
                print(f"Length: {length}, Speed: {speed}, Start: {start_angle}°, "
                      f"Stop: {stop_angle}°, Time: {timestamp}, CRC: {crc}")
            
            return list(zip(angle, distance, confidence))
            
        except struct.error:
            if self.debug:
                print("Error unpacking LIDAR data packet")
            return []

    def convert_to_cartesian(self, measurements):
        """Convert polar measurements to cartesian coordinates
        
        Args:
            measurements (list): List of tuples (angle, distance, confidence)
            
        Returns:
            tuple: Numpy arrays (x, y, confidence)
        """
        if not measurements:
            return np.array([]), np.array([]), np.array([])
            
        # Unpack the tuples
        angle = np.array([measurement[0] for measurement in measurements])
        distance = np.array([measurement[1] for measurement in measurements])
        confidence = np.array([measurement[2] for measurement in measurements])
        
        # Convert to cartesian coordinates in meters
        x = np.sin(np.radians(angle)) * (distance / 1000.0)
        y = np.cos(np.radians(angle)) * (distance / 1000.0)
        
        return x, y, confidence

    def process_scan_data(self, measurements, scan_direction=0):
        """Process measurements into a format ready for ROS LaserScan
        
        Args:
            measurements (list): List of tuples (angle, distance, confidence)
            scan_direction (float): Scan direction
            
        Returns:
            tuple: (angles_rad, distances_m, confidences)
                angles_rad: Numpy array of angles in radians
                distances_m: Numpy array of distances in meters
                confidences: Numpy array of confidence values
        """
        if not measurements:
            return np.array([]), np.array([]), np.array([])
            
        # Sort measurements by angle
        measurements.sort(key=lambda x: x[0])
        
        # Extract values
        angles = np.array([m[0] for m in measurements])
        distances = np.array([m[1] for m in measurements]) / 1000.0  # convert to meters
        confidences = np.array([m[2] for m in measurements])
        
        # Convert degrees to radians and correct the direction if needed
        # if flip_direction:
        #     # Flip the scan direction: In ROS, 0 radians is forward and increases counterclockwise
        #     angles_rad = np.radians((scan_direction - angles) % 360.0)
        # else:
        #     angles_rad = np.radians(angles % 360.0)

        angles_rad = np.radians((scan_direction - angles) % 360.0)
        
        return angles_rad, distances, confidences

    @staticmethod
    def is_header_valid(data):
        """Check if the data starts with a valid header
        
        Args:
            data (bytes): Raw binary data
            
        Returns:
            bool: True if the header is valid, False otherwise
        """
        return len(data) >= 2 and data[0] == 0x54 and data[1] == 0x2C