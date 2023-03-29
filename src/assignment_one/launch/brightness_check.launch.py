from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Add the brightness_check Node.
        Node(
            package = 'assignment_one', 
            executable = 'brightness_check',
            parameters = [
                {"threshold": 120},
            ],
        ),
        # Add the cam2image Node.
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
            parameters = [
                {"depth": 1},
                {"history": "keep_last"},
            ],
        ),
        # Add the showimage Node.
        Node(
            package = 'image_tools',
            executable = 'showimage',
        ),
    ]

    return LaunchDescription(nodes)