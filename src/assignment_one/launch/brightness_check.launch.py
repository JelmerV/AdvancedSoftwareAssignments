from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Add the brightness_check Node.
        Node(
            package = 'assignment_one', 
            executable = 'brightness_check',
        ),
        # Add the cam2image Node.
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
        ),
    ]

    return LaunchDescription(nodes)