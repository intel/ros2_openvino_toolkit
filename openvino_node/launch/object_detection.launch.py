import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    print(get_package_share_directory('openvino_ros')+'/config/object_detection.yaml')
    container = ComposableNodeContainer(
            node_name='vision_pipeline',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='openvino_ros',
                    node_plugin='openvino::OpenVINOFactory',
                    node_name='detect',
                    parameters=[get_package_share_directory('openvino_ros')+'/config/object_detection.yaml'])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
