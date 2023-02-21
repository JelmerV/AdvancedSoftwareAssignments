from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    # add the light_check node
    node1 = launch_ros.actions.Node(
        namespace = 'assign1', 
        package = 'assign1', 
        executable = 'light_check',
    )

    return LaunchDescription([node1])