import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    talker = Node(
        name = 'talker',
        namespace = 'throughput_performance_test',
        package = 'throughput_performance_test',
        executable = 'talker_node',
        output = 'screen'
    )
    image_processing = ComposableNodeContainer(
            name = 'image_processing',
            namespace = 'throughput_performance_test',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions=[
                ComposableNode(
                    name = 'rectifier',
                    namespace = 'throughput_performance_test',
                    package = 'image_proc',
                    plugin = 'image_proc::RectifyNode')
            ],
            output = 'screen'
    )
    listener = Node(
        name = 'listener',
        namespace = 'throughput_performance_test',
        package = 'throughput_performance_test',
        executable = 'listener_node',
        parameters = [{'topic': 'image'}],
        output = 'screen'
    )
    return launch.LaunchDescription([talker, #image_processing, 
    listener])
