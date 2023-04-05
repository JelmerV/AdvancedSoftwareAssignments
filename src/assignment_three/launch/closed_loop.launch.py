from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Add the 'light_position' Node with the camera enabled and the 'image' topic remapped to 'webcam_input'.
        Node(
            package = 'assignment_one', 
            executable = 'light_position',
            parameters = [
                {'threshold': 200},
            ],
            remappings=[
                ('image', 'webcam_input'),
            ],
        ),
        # Add the 'jiwy_trajectory' Node.
        Node(
            package = 'assignment_one', 
            executable = 'jiwy_trajectory',
            parameters = [
                {'use_camera': True},
            ],
        ),
        Node(
            package = 'closed_loop',
            executable = 'controller',
            parameters = [
                {'tau_s': 1.0}
            ]
            remappings = [
                ('cog_input', 'image_cog')
            ]
        )
        # Add the 'cam2image' Node with the 'image' topic remapped to 'webcam_input'.
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
            parameters = [
                {"depth": 1},
                {"history": "keep_last"},
            ],
            remappings=[
                ('image', 'webcam_input'),
            ],
        ),
        # Add a 'showimage' Node with the 'image' topic remapped to 'mono_image'.
        Node(
            package = 'image_tools', 
            executable = 'showimage',
            name='showimage_mono',
            remappings=[
                ('image', 'mono_image'),
            ],
        ),
        # Add a 'showimage' Node to show the Jiwy simulator output with the 'image' topic remapped to 'moving_camera_output'.
        Node(
            package = 'image_tools', 
            executable = 'showimage',
            name='showimage_jiwy',
            remappings=[
                ('image', 'moving_camera_output'),
            ],
        ),
        # Add the 'jiwy_simulator' Node.
        Node(
            package = 'jiwy_simulator', 
            executable = 'jiwy_simulator',
        ),
    ]
    return LaunchDescription(nodes)