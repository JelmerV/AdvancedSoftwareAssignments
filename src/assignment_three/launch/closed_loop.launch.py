from launch import LaunchDescription
from launch_ros.actions import Node

# import os
# os.system('../src/xenomai-ros2-framework/build/jiwy')

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
                {'tau_s': 1.0},
            ],
            remappings = [
                ('cog_input', 'image_cog'),
            ]
        ),
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

        # listens to setpoint for framework
        Node(
            package = 'ros2-xenomai',
            executable = 'listener',
            name = 'xeno_listener_pan',
            parameters = [
                {'topicName': 'setpoint_pan'},
                {'xdppPort': 10}
            ]
        ),
        # Node(
        #     package = 'ros2-xenomai',
        #     executable = 'listener',
        #     name = 'xeno_listener_tilt',
        #     parameters = [
        #         {'topicName': 'setpoint_tilt'},
        #         {'xdppPort': 11},
        #     ]
        # ),

        # publishes measured position
        Node(
            package = 'ros2-xenomai',
            executable = 'talker',
            name = 'xeno_talker_pan',
            parameters = [
                {'topicName': 'position_pan'},
                {'xdppPort': 20},
            ],
        ),
        # Node(
        #     package = 'ros2-xenomai',
        #     executable = 'talker',
        #     name = 'xeno_talker_tilt',
        #     parameters = [
        #         {'topicName': 'position_tilt'},
        #         {'xdppPort': 21},
        #     ],
        # )
    ]
    return LaunchDescription(nodes)