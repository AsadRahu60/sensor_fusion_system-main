"""
═══════════════════════════════════════════════════════════════════════
FILE PURPOSE: sensor_fusion_system.launch.py
═══════════════════════════════════════════════════════════════════════

WHAT IT DOES:
- Launches all 4 nodes simultaneously
- Loads parameters
- Sets up node relationships

HOW TO USE:
- ros2 launch sensor_fusion_system sensor_fusion_system.launch.py

WHY YOU NEED IT:
- Convenient way to start entire system
- Ensures correct configuration
═══════════════════════════════════════════════════════════════════════
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config file path
    config = os.path.join(
        get_package_share_directory('sensor_fusion_system'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        # IMU Sensor
        Node(
            package='sensor_fusion_system',
            executable='imu_sensor_node',
            name='imu_sensor',
            parameters=[config],
            output='screen'
        ),
        
        # Lidar Sensor
        Node(
            package='sensor_fusion_system',
            executable='lidar_sensor_node',
            name='lidar_sensor',
            parameters=[config],
            output='screen'
        ),
        
        # Fusion Node
        Node(
            package='sensor_fusion_system',
            executable='fusion_node',
            name='fusion',
            parameters=[config],
            output='screen'
        ),
        
        # Safety Monitor
        Node(
            package='sensor_fusion_system',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen'
        ),
    ])