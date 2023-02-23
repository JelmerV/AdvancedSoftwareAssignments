from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # takes image and publishes setpoint and thresholded image
        Node(
            package = 'assign1', 
            executable = 'jiwy_trajectory',
            namespace='jiwy',
            parameters = [
                {'threshold': 200},
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
            name='showimage_thres',
            namespace='jiwy',
            remappings=[
                ('image', 'image_thres'),
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