#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the path to config files
    config_dir = os.path.join(get_package_share_directory('hexapod_bringup'), 'config')
    
    # Declare launch arguments for enabling/disabling components
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='True',
        description='Enable RPLiDAR A1'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='True',
        description='Enable RPi Camera'
    )

    # Load configuration files
    with open(os.path.join(config_dir, 'servo_config.yaml'), 'r') as f:
        servo_config = yaml.safe_load(f)
    
    with open(os.path.join(config_dir, 'sensor_config.yaml'), 'r') as f:
        sensor_config = yaml.safe_load(f)

    return LaunchDescription([
        # Launch arguments
        enable_lidar_arg,
        enable_camera_arg,

        # Servo Controller Node
        Node(
            package='hexapod_hardware',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
            parameters=[
                servo_config,
                {
                    'i2c_bus': 1,  # I2C bus for PCA9685
                    'pca9685_address': 0x40,  # Default I2C address for PCA9685
                    'pwm_frequency': 50,  # 50Hz for servo motors
                    'servo_min_pulse': 500,  # Minimum pulse width in microseconds
                    'servo_max_pulse': 2500  # Maximum pulse width in microseconds
                }
            ]
        ),

        # IMU Node
        Node(
            package='hexapod_hardware',
            executable='imu_interface',
            name='imu_node',
            output='screen',
            parameters=[
                sensor_config,
                {
                    'i2c_bus': 1,
                    'imu_address': 0x68,  # Default address for MPU6886
                    'update_rate': 100.0,  # 100Hz update rate
                    'gyro_range': 2000,  # deg/s
                    'accel_range': 16  # g
                }
            ]
        ),

        # GPS Node
        Node(
            package='hexapod_hardware',
            executable='gps_interface',
            name='gps_node',
            output='screen',
            parameters=[
                sensor_config,
                {
                    'port': '/dev/ttyAMA0',
                    'baud': 9600,
                    'update_rate': 1.0,  # 1Hz update rate
                    'frame_id': 'gps_link'
                }
            ]
        ),

        # LiDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'scan_mode': 'Standard',
                'angle_compensate': True,
                'scan_frequency': 10.0  # 10Hz scan frequency
            }],
            condition=LaunchConfiguration('enable_lidar')
        ),

        # Camera Node
        Node(
            package='hexapod_hardware',
            executable='camera_interface',
            name='camera_node',
            output='screen',
            parameters=[
                sensor_config,
                {
                    'width': 640,
                    'height': 480,
                    'framerate': 30,
                    'enable_night_vision': True,
                    'auto_exposure': True,
                    'exposure_time': 1000,  # microseconds
                    'gain': 1.0
                }
            ],
            condition=LaunchConfiguration('enable_camera')
        ),

        # Health Monitor Node
        Node(
            package='hexapod_perception',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'check_frequency': 1.0,  # Check system health at 1Hz
                'signal_strength_threshold': 70.0,  # 70% signal strength threshold
                'temperature_threshold': 80.0,  # 80°C temperature threshold
                'stall_timeout': 45.0,  # 45 seconds timeout for stall detection
                'enable_diagnostics': True
            }]
        )
    ])