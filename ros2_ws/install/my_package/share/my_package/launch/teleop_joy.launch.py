from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }]
        ),
        
        # Teleop twist joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,      # Left stick up/down
                'scale_linear.x': 1.0,   # Max linear speed
                'axis_angular.yaw': 0,   # Left stick left/right
                'scale_angular.yaw': 1.0, # Max angular speed
                'enable_button': 4,      # LB button (hold to enable)
                'enable_turbo_button': 5 # RB button (turbo mode)
            }]
        )
    ])