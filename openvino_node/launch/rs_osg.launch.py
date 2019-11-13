import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    print(get_package_share_directory('openvino_node')+'/config/object_segmentation.yaml')
    container = ComposableNodeContainer(
            node_name='vision_pipeline',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense_ros',
                    node_plugin='realsense::RealSenseNodeFactory',
                    node_name='realsense',
                    parameters=[get_package_share_directory('realsense_examples')+'/config/d435i.yaml'],
                    extra_arguments=[{'use_intra_process_comms':'true'}]),
                ComposableNode(
                    package='openvino_ros',
                    node_plugin='openvino::OpenVINOFactory',
                    node_name='object_segmentation',
                    remappings=[('/rdk/openvino/segmented_objects', '/openvino/segmented_objects'), ('/rdk/openvino/image_raw', '/camera/color/image_raw')],
                    parameters=[get_package_share_directory('openvino_node')+'/config/object_segmentation.yaml'],
                    extra_arguments=[{'use_intra_process_comms':'true'}])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
