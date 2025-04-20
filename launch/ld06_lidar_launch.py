from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the LD06 LIDAR'
    )
    
    scan_direction_arg = DeclareLaunchArgument(
        'scan_direction',
        default_value='90.0',
        description='Set the direction of the Lidar sensor'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_base',
        description='Frame ID for the laser scan'
    )
    
    # LD06 LIDAR node
    ld06_node = Node(
        package='ld06_lidar',
        executable='ld06_node',
        name='ld06_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'scan_direction': LaunchConfiguration('scan_direction'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        scan_direction_arg,
        frame_id_arg,
        ld06_node
    ])