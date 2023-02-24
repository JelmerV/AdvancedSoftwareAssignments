from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Add the brightness_check Node.
    node_brightness = Node(
        namespace = 'assignment_one', 
        package = 'assignment_one', 
        executable = 'brightness_check',
    )

    return LaunchDescription([node_brightness])