from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Add the cam2image Node.
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
            parameters = [
                {"depth": 1},
                {"history": "keep_last"},
            ],
        ),
        # Add the light_position Node.
        Node(
            package = 'assignment_one', 
            executable = 'light_position',
            parameters = [
                {"threshold": 200},
            ],
        ),
        # Add the showimage Node.
        Node(
            package = 'image_tools',
            executable = 'showimage',
            remappings=[
                ('image', 'mono_image'),
            ],
        ),
    ]

    return LaunchDescription(nodes)