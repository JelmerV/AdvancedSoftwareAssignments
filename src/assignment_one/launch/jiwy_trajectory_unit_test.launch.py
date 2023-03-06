from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Add the 'jiwy_trajectory' Node.
        Node(
            package = 'assignment_one', 
            executable = 'jiwy_trajectory',
            namespace='jiwy',
            parameters = [
                {'use_camera': False},
                {'interval': 1000},
            ],
        ),
        # Add the 'cam2image' Node with the 'image' topic remapped to 'webcam_input'.
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
            namespace='jiwy',
            parameters=[
                {"depth": 1},
                {"history": "keep_last"},
            ],
            remappings=[
                ('image', 'webcam_input'),
            ],
        ),
        # Add a 'showimage' Node to show the Jiwy simulator output with the 'image' topic remapped to 'moving_camera_output'.
        Node(
            package = 'image_tools', 
            executable = 'showimage',
            name='showimage_jiwy',
            namespace='jiwy',
            remappings=[
                ('image', 'moving_camera_output'),
            ],
        ),
        # Add the 'jiwy_simulator' Node.
        Node(
            package = 'jiwy_simulator', 
            executable = 'jiwy_simulator',
            namespace='jiwy',
        ),
    ]
    return LaunchDescription(nodes)