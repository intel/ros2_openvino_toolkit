import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_yaml = os.path.join(get_package_share_directory('openvino_node'), 'param',
                                'pipeline_composite_object_topic.yaml')
    container = ComposableNodeContainer(
        node_name='vision_pipeline',
        node_namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense_ros',
                node_plugin='realsense::RealSenseNodeFactory',
                node_name='realsense',
                parameters=[get_package_share_directory('realsense_examples')+'/config/d435i.yaml'],
                extra_arguments=[{'use_intra_process_comms':'true'}]),
            ComposableNode(
                package='openvino_node',
                node_plugin='ComposablePipeline',
                node_name='composable_pipeline',
                parameters=[{"config":default_yaml}],
                remappings=[
                ('/openvino_toolkit/object/detected_objects',
                 '/ros2_openvino_toolkit/detected_objects'),
                ('/openvino_toolkit/object/images', '/ros2_openvino_toolkit/image_rviz')],
                extra_arguments=[{'use_intra_process_comms':'true'}]
            )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
