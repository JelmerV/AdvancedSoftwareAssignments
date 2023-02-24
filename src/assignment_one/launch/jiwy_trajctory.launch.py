from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Takes image and publishes setpoint and thresholded image
        Node(
            package = 'assignment_one', 
            executable = 'jiwy_trajectory',
            namespace='jiwy',
            parameters = [
                {'threshold': 200},
                {'use_camera': True},
            ],
            remappings=[
                ('image', 'webcam_input'),
            ],
        ),
        # publish webcam images to the webcam_input topic
        Node(
            package = 'image_tools', 
            executable = 'cam2image',
            namespace='jiwy',
            remappings=[
                ('image', 'webcam_input'),
            ],
        ),
        # show the thresholded image
        Node(
            package = 'image_tools', 
            executable = 'showimage',
            name='showimage_mono',
            namespace='jiwy',
            remappings=[
                ('image', 'mono_image'),
            ],
        ),
        # show the jiwy sim output
        Node(
            package = 'image_tools', 
            executable = 'showimage',
            name='showimage_jiwy',
            namespace='jiwy',
            remappings=[
                ('image', 'moving_camera_output'),
            ],
        ),
        # start the jiwy simulator
        Node(
            package = 'jiwy_simulator', 
            executable = 'jiwy_simulator',
            namespace='jiwy',
        ),
    ]
    return LaunchDescription(nodes)