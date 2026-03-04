"""Launch the motor bridge node with configurable parameters."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='ESP32 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
                              description='Serial baud rate'),
        DeclareLaunchArgument('use_pid', default_value='true',
                              description='Enable PID (false = open-loop)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Broadcast odom→base_link TF'),
        DeclareLaunchArgument('v_max', default_value='0.5',
                              description='Robot max speed at PWM=255 (m/s)'),

        Node(
            package='my_package',
            executable='motor_bridge_node',
            name='motor_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate':   LaunchConfiguration('baud_rate'),
                'use_pid':     LaunchConfiguration('use_pid'),
                'publish_tf':  LaunchConfiguration('publish_tf'),
                'v_max':       LaunchConfiguration('v_max'),
            }],
        ),
    ])