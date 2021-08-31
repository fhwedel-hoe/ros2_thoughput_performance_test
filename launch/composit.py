import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_node_descriptions = [
        ComposableNode(
            name = 'talker',
            namespace = 'throughput_performance_test',
            package = 'throughput_performance_test',
            plugin = 'throughput_performance_test::Talker'
        ),
        ComposableNode(
            name = 'listener',
            namespace = 'throughput_performance_test',
            package = 'throughput_performance_test',
            plugin = 'throughput_performance_test::Listener',
            parameters = [{'topic': 'image'}]
        )
    ]
    """
    composable_node_descriptions.append(
        ComposableNode(
            name = 'rectifier',
            namespace = 'throughput_performance_test',
            package = 'image_proc',
            plugin = 'image_proc::RectifyNode'
        )
    )
    """
    image_processing = ComposableNodeContainer(
            name = 'image_processing',
            namespace = 'throughput_performance_test',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = composable_node_descriptions,
            output = 'screen'
    )

    return launch.LaunchDescription([image_processing])
